// Bridge tussen ROS 2 en het ESP32 general driver board van de Wave Rover.
//
// C++-versie van wave_rover_base/esp32_bridge.py: zelfde topics, parameters en rekenregels
// (vergeleken met scripts/compare_esp32_bridge.py). De Python-versie gebruikte in rust ±11%
// van een kern, vooral doordat pyserial de seriële poort byte per byte leest (29-10-2026).
//
// - /cmd_vel (TwistStamped) wordt omgezet naar wielcommando's {"T":1,"L":..,"R":..}.
//   De motoren hebben geen encoders: de snelheid vooruit is open-loop. De draaisnelheid wordt
//   bijgestuurd met de gyroscoop (PI-regelaar, yaw_kp/yaw_ki).
// - Komt er langer dan cmd_timeout geen /cmd_vel binnen, dan worden de motoren gestopt.
// - De IMU (T:126 -> antwoord T:1002) en de chassisfeedback (T:130 -> T:1001) worden
//   periodiek opgevraagd en gepubliceerd als /imu/data_raw en /battery.
// - Het nulpunt van de gyroscoop wordt bijgehouden terwijl de motoren stilstaan en
//   afgetrokken (het board geeft bij stilstand tot 22 °/s). /imu/data_raw komt pas zodra er
//   een nulpunt is (~2 s stilstand na het starten).
// Een aparte thread leest de poort (poll + read per blok), zodat de IMU-gegevens meteen
// doorkomen; schrijven gebeurt alleen vanuit de ROS-thread.

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace
{

constexpr double G = 9.80665;
constexpr int GYRO_BIAS_START = 40;  // metingen bij stilstand voor het eerste nulpunt (2 s bij 20 Hz)

double monotonic()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

double radians(double degrees) {return degrees * M_PI / 180.0;}

// Getal zoals json.dumps(round(x, 3)) in Python: "0.25", "-0.1", "0.0".
std::string fmt3(double x)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.3f", x);
  std::string s(buf);
  while (s.back() == '0' && s[s.size() - 2] != '.') {
    s.pop_back();
  }
  return s;
}

// Eén regel van het board, bv. {"T":1002,"r":1.5,"gz":-0.3}: de getallen per sleutel. Het
// board stuurt platte objecten; tekst en true/false/null worden overgeslagen. Klopt de regel
// niet (afgebroken, rommel, genest), dan false en wordt ze genegeerd, zoals json.loads.
bool parse_line(const std::string & line, std::map<std::string, double> & out)
{
  out.clear();
  const char * p = line.c_str();
  auto skip_ws = [&p]() {
      while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
        ++p;
      }
    };
  skip_ws();
  if (*p++ != '{') {
    return false;
  }
  skip_ws();
  if (*p == '}') {
    ++p;
    skip_ws();
    return *p == '\0';
  }
  while (true) {
    skip_ws();
    if (*p++ != '"') {
      return false;
    }
    const char * key = p;
    while (*p && *p != '"') {
      ++p;
    }
    if (!*p) {
      return false;
    }
    std::string name(key, p - key);
    ++p;
    skip_ws();
    if (*p++ != ':') {
      return false;
    }
    skip_ws();
    if (*p == '"') {
      for (++p; *p && *p != '"'; ++p) {
        if (*p == '\\' && p[1]) {
          ++p;
        }
      }
      if (!*p) {
        return false;
      }
      ++p;
    } else if (!std::strncmp(p, "true", 4) || !std::strncmp(p, "null", 4)) {
      p += 4;
    } else if (!std::strncmp(p, "false", 5)) {
      p += 5;
    } else {
      char * end = nullptr;
      const double value = std::strtod(p, &end);
      if (end == p) {
        return false;
      }
      out[name] = value;
      p = end;
    }
    skip_ws();
    if (*p == ',') {
      ++p;
    } else if (*p == '}') {
      ++p;
      skip_ws();
      return *p == '\0';
    } else {
      return false;
    }
  }
}

speed_t baud_constant(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: throw std::runtime_error("baudrate " + std::to_string(baud) + " niet ondersteund");
  }
}

// Seriële poort openen zoals pyserial: 8N1, geen flow control, DTR en RTS hoog, oude invoer weg.
int open_serial(const std::string & port, int baud)
{
  const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    throw std::runtime_error(port + ": " + std::strerror(errno));
  }
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    const std::string err = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error(port + ": " + err);
  }
  cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  cfsetispeed(&tty, baud_constant(baud));
  cfsetospeed(&tty, baud_constant(baud));
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    const std::string err = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error(port + ": " + err);
  }
  // Schrijven blokkeert (zoals pyserial); lezen gebeurt pas na poll().
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);
  int bits = TIOCM_DTR | TIOCM_RTS;
  ioctl(fd, TIOCMBIS, &bits);  // faalt op een pseudo-terminal (test); niet erg
  tcflush(fd, TCIFLUSH);
  return fd;
}

}  // namespace

class Esp32Bridge : public rclcpp::Node
{
public:
  Esp32Bridge()
  : Node("esp32_bridge")
  {
    const auto port = declare_parameter<std::string>("port", "/dev/esp32");
    const int baud = static_cast<int>(declare_parameter<int64_t>("baud", 115200));
    // Effectieve wielafstand (m). Bij skid-steer schuiven de wielen zijdelings bij het
    // draaien; daardoor is meer verschil tussen links en rechts nodig dan de echte
    // wielafstand (0,12 m) doet vermoeden.
    track_width_ = declare_parameter<double>("track_width", 0.3);
    // Snelheid (m/s) die overeenkomt met wielwaarde max_wheel_cmd.
    speed_at_max_cmd_ = declare_parameter<double>("speed_at_max_cmd", 1.25);
    max_wheel_cmd_ = declare_parameter<double>("max_wheel_cmd", 0.5);
    // Kleinste wielwaarde waarbij de motoren op gang komen; kleinere commando's worden
    // lineair geschaald naar [min_wheel_cmd, max_wheel_cmd].
    min_wheel_cmd_ = declare_parameter<double>("min_wheel_cmd", 0.1);
    // Skid-steer heeft in bochten meer kracht nodig (de wielen schuiven zijdelings): het
    // minimum loopt op tot deze waarde naarmate de bocht scherper is.
    min_turn_wheel_cmd_ = declare_parameter<double>("min_turn_wheel_cmd", 0.25);
    // Minimum bij draaien op de plaats.
    min_spin_wheel_cmd_ = declare_parameter<double>("min_spin_wheel_cmd", 0.17);
    // Correctie van de draaisnelheid (scripts/calibrate_motion.py).
    spin_gain_ = declare_parameter<double>("spin_gain", 1.6);
    curve_gain_ = declare_parameter<double>("curve_gain", 0.9);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
    const auto imu_rate = declare_parameter<double>("imu_rate", 20.0);
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
    // Tijdconstante (s) van het lopende gemiddelde voor het gyronulpunt.
    const auto gyro_bias_time = declare_parameter<double>("gyro_bias_time", 10.0);
    // Draaisnelheid bijsturen met de gyro: correctie = kp * fout + integraal(ki * fout),
    // begrensd op yaw_i_max (rad/s). yaw_loop false = alleen de tabel.
    yaw_loop_ = declare_parameter<bool>("yaw_loop", true);
    yaw_kp_ = declare_parameter<double>("yaw_kp", 0.5);
    yaw_ki_ = declare_parameter<double>("yaw_ki", 2.0);
    yaw_i_max_ = declare_parameter<double>("yaw_i_max", 2.5);
    gyro_bias_alpha_ = 1.0 / (imu_rate * gyro_bias_time);
    last_motion_ = monotonic();  // motoren draaiden; de eerste seconde niet meten

    // Afstelling tijdens het rijden aanpassen (ros2 param set /esp32_bridge ...).
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return on_parameters(params);});

    fd_ = open_serial(port, baud);
    RCLCPP_INFO(get_logger(), "Verbonden met ESP32 op %s (%d baud)", port.c_str(), baud);

    // Continue feedbackstroom uit: we vragen zelf op, zodat de seriële lijn niet overloopt.
    send("{\"T\":131,\"cmd\":0}");
    stop_motors();

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {on_cmd_vel(*msg);});

    imu_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / imu_rate), [this]() {send("{\"T\":126}");});
    battery_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {send("{\"T\":130}");});
    timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {check_cmd_timeout();});

    reader_ = std::thread([this]() {read_loop();});
  }

  ~Esp32Bridge() override
  {
    running_ = false;
    if (reader_.joinable()) {
      reader_.join();
    }
    stop_motors();
    ::close(fd_);
  }

private:
  void send(const std::string & msg)
  {
    const std::string line = msg + "\n";
    std::lock_guard<std::mutex> lock(write_mutex_);
    size_t done = 0;
    while (done < line.size()) {
      const ssize_t n = ::write(fd_, line.data() + done, line.size() - done);
      if (n < 0 && errno == EINTR) {
        continue;
      }
      if (n <= 0) {
        serial_failed(std::string("schrijven: ") + std::strerror(errno));
        return;
      }
      done += static_cast<size_t>(n);
    }
  }

  // Poort weg (USB los): melden en stoppen, zoals de Python-versie (die dan crasht).
  void serial_failed(const std::string & what)
  {
    if (!failed_.exchange(true)) {
      RCLCPP_ERROR(get_logger(), "Seriële fout: %s", what.c_str());
      rclcpp::shutdown();
    }
  }

  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    const std::map<std::string, double *> tunable = {
      {"min_wheel_cmd", &min_wheel_cmd_}, {"min_turn_wheel_cmd", &min_turn_wheel_cmd_},
      {"min_spin_wheel_cmd", &min_spin_wheel_cmd_}, {"spin_gain", &spin_gain_},
      {"curve_gain", &curve_gain_}, {"yaw_kp", &yaw_kp_}, {"yaw_ki", &yaw_ki_},
      {"yaw_i_max", &yaw_i_max_}};
    for (const auto & p : params) {
      if (p.get_name() == "yaw_loop") {
        yaw_loop_ = p.as_bool();
      } else if (auto it = tunable.find(p.get_name()); it != tunable.end()) {
        *it->second = p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
          static_cast<double>(p.as_int()) : p.as_double();
      } else {
        continue;
      }
      RCLCPP_INFO(get_logger(), "%s = %s", p.get_name().c_str(), p.value_to_string().c_str());
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void stop_motors() {send("{\"T\":1,\"L\":0,\"R\":0}");}

  void on_cmd_vel(const geometry_msgs::msg::TwistStamped & stamped)
  {
    const auto & msg = stamped.twist;
    const double half = track_width_ / 2.0;
    double wz = msg.angular.z + yaw_correction(msg.linear.x, msg.angular.z);
    wz *= angular_gain(msg.linear.x, msg.angular.z * half);
    double left = msg.linear.x - wz * half;
    double right = msg.linear.x + wz * half;

    to_wheel_cmds(left, right);
    send("{\"T\":1,\"L\":" + fmt3(left) + ",\"R\":" + fmt3(right) + "}");
    if (left != 0.0 || right != 0.0) {
      last_motion_ = monotonic();
    }
    last_cmd_time_ = monotonic();
  }

  // PI-correctie (rad/s) zodat de gyro meet wat gevraagd wordt.
  double yaw_correction(double forward, double wz_ref)
  {
    const double now = monotonic();
    const double dt = yaw_last_ ? std::min(0.1, now - *yaw_last_) : 0.05;
    yaw_last_ = now;
    if (!yaw_loop_ || (std::abs(forward) < 1e-3 && std::abs(wz_ref) < 1e-3)) {
      yaw_i_ = 0.0;  // stilstand: geen opgebouwde correctie meenemen
      return 0.0;
    }
    if (now - gyro_time_ > 0.3) {
      return 0.0;  // geen verse gyrometing (nog geen nulpunt of IMU weg)
    }
    const double error = wz_ref - gyro_z_;
    yaw_i_ = std::max(-yaw_i_max_, std::min(yaw_i_max_, yaw_i_ + yaw_ki_ * error * dt));
    double correction = yaw_kp_ * error + yaw_i_;
    if (wz_ref * (wz_ref + correction) < 0) {
      // Nooit tegen de gevraagde richting in: trager dan het minimum van de motoren kan
      // niet, dan liever even 0 (en de integraal niet verder laten oplopen).
      yaw_i_ -= wz_ref + correction;
      correction = -wz_ref;
    }
    return correction;
  }

  // Vloeiend van curve_gain (vooral vooruit) naar spin_gain (op de plaats draaien).
  double angular_gain(double forward, double turn) const
  {
    const double total = std::abs(forward) + std::abs(turn);
    if (total < 1e-6) {
      return 1.0;
    }
    // Kwadratisch: de grote correctie geldt pas bij (bijna) zuiver draaien op de plaats.
    const double spin = std::pow(std::abs(turn) / total, 2);
    return curve_gain_ * (1.0 - spin) + spin_gain_ * spin;
  }

  // Wielsnelheden (m/s) -> wielwaarden voor T:1. Het snelste wiel krijgt minstens een minimum
  // om de motoren op gang te krijgen: min_wheel_cmd rechtdoor, oplopend tot
  // min_turn_wheel_cmd in een scherpe bocht (één wiel stil) en min_spin_wheel_cmd bij draaien
  // op de plaats. Beide wielen met dezelfde factor, zodat de bochtstraal behouden blijft.
  void to_wheel_cmds(double & left, double & right) const
  {
    const double scale = max_wheel_cmd_ / speed_at_max_cmd_;
    left *= scale;
    right *= scale;
    const double fastest = std::max(std::abs(left), std::abs(right));
    if (fastest < 1e-3) {
      left = right = 0.0;
      return;
    }
    // 0 = rechtdoor, 1 = één wiel staat stil, 2 = draaien op de plaats.
    const double turn = std::abs(left - right) / fastest;
    double minimum;
    if (turn <= 1.0) {
      minimum = min_wheel_cmd_ + (min_turn_wheel_cmd_ - min_wheel_cmd_) * turn;
    } else {
      minimum = min_turn_wheel_cmd_ +
        (min_spin_wheel_cmd_ - min_turn_wheel_cmd_) * std::min(1.0, turn - 1.0);
    }
    double fastest_cmd = std::min(max_wheel_cmd_, fastest);
    fastest_cmd = minimum + (max_wheel_cmd_ - minimum) * fastest_cmd / max_wheel_cmd_;
    const double factor = fastest_cmd / fastest;
    left *= factor;
    right *= factor;
  }

  void check_cmd_timeout()
  {
    if (!last_cmd_time_) {
      return;
    }
    if (monotonic() - *last_cmd_time_ > cmd_timeout_) {
      stop_motors();
      last_cmd_time_.reset();
      yaw_i_ = 0.0;
    }
  }

  void read_loop()
  {
    std::string buffer;
    char chunk[512];
    while (running_) {
      pollfd pfd{fd_, POLLIN, 0};
      const int ready = ::poll(&pfd, 1, 100);  // elke 0,1 s kijken of we moeten stoppen
      if (ready < 0 && errno == EINTR) {
        continue;
      }
      if (ready < 0 || (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
        serial_failed("poort weg");
        return;
      }
      if (ready == 0) {
        continue;
      }
      const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
      if (n < 0 && (errno == EINTR || errno == EAGAIN)) {
        continue;
      }
      if (n <= 0) {
        serial_failed(n < 0 ? std::strerror(errno) : "gereed om te lezen maar geen gegevens");
        return;
      }
      buffer.append(chunk, static_cast<size_t>(n));
      size_t start = 0;
      for (size_t nl; (nl = buffer.find('\n', start)) != std::string::npos; start = nl + 1) {
        handle_line(buffer.substr(start, nl - start));
      }
      buffer.erase(0, start);
      if (buffer.size() > 4096) {
        buffer.clear();  // geen regeleinde meer: rommel
      }
    }
  }

  void handle_line(const std::string & line)
  {
    if (!parse_line(line, fields_)) {
      return;
    }
    const auto t = fields_.find("T");
    if (t == fields_.end()) {
      return;
    }
    if (t->second == 1002) {
      publish_imu();
    } else if (t->second == 1001) {
      publish_battery();
    }
  }

  bool field(const char * name, double & value) const
  {
    const auto it = fields_.find(name);
    if (it == fields_.end()) {
      return false;
    }
    value = it->second;
    return true;
  }

  // Gyronulpunt bijwerken met een meting (°/s), alleen als de motoren stilstaan.
  void update_gyro_bias(const std::array<double, 3> & gyro)
  {
    if (monotonic() - last_motion_ < 1.0) {
      return;
    }
    if (gyro_samples_ < GYRO_BIAS_START) {
      // Eerst een gewoon gemiddelde, daarna een lopend gemiddelde (volgt temperatuur).
      const int n = gyro_samples_;
      for (int i = 0; i < 3; ++i) {
        gyro_bias_[i] = (gyro_bias_[i] * n + gyro[i]) / (n + 1);
      }
      if (++gyro_samples_ == GYRO_BIAS_START) {
        RCLCPP_INFO(
          get_logger(), "Gyronulpunt: x %.1f, y %.1f, z %.1f °/s",
          gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
      }
    } else if (std::abs(gyro[0] - gyro_bias_[0]) < 25.0 &&
      std::abs(gyro[1] - gyro_bias_[1]) < 25.0 && std::abs(gyro[2] - gyro_bias_[2]) < 25.0)
    {
      // Grote uitschieters: iemand tilt of duwt de robot; niet meetellen.
      for (int i = 0; i < 3; ++i) {
        gyro_bias_[i] += gyro_bias_alpha_ * (gyro[i] - gyro_bias_[i]);
      }
    }
  }

  void publish_imu()
  {
    double r, p, y, ax, ay, az, gx, gy, gz;
    if (!field("r", r) || !field("p", p) || !field("y", y) || !field("ax", ax) ||
      !field("ay", ay) || !field("az", az) || !field("gx", gx) || !field("gy", gy) ||
      !field("gz", gz))
    {
      return;  // onvolledig bericht
    }
    update_gyro_bias({gx, gy, gz});
    if (gyro_samples_ < GYRO_BIAS_START) {
      return;  // zonder nulpunt liever geen IMU dan een draaiende robot
    }
    gx -= gyro_bias_[0];
    gy -= gyro_bias_[1];
    gz -= gyro_bias_[2];
    // Voor de draairegeling: licht gefilterd (ruis ±4,5 °/s bij 20 Hz).
    gyro_z_ = 0.5 * gyro_z_ + 0.5 * radians(gz);
    gyro_time_ = monotonic();

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = imu_frame_;

    const double cr = std::cos(radians(r) / 2), sr = std::sin(radians(r) / 2);
    const double cp = std::cos(radians(p) / 2), sp = std::sin(radians(p) / 2);
    const double cy = std::cos(radians(y) / 2), sy = std::sin(radians(y) / 2);
    msg.orientation.x = sr * cp * cy - cr * sp * sy;
    msg.orientation.y = cr * sp * cy + sr * cp * sy;
    msg.orientation.z = cr * cp * sy - sr * sp * cy;
    msg.orientation.w = cr * cp * cy + sr * sp * sy;
    // Yaw komt van de magnetometer en is binnenshuis onbetrouwbaar: grote onzekerheid.
    msg.orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0};

    // Gyroscoop in °/s (min het nulpunt), versnelling in mg.
    msg.angular_velocity.x = radians(gx);
    msg.angular_velocity.y = radians(gy);
    msg.angular_velocity.z = radians(gz);
    // Gemeten ruis bij stilstand: ±3 °/s met pieken tot 16 °/s -> std ~0.1 rad/s.
    msg.angular_velocity_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
    msg.linear_acceleration.x = ax / 1000.0 * G;
    msg.linear_acceleration.y = ay / 1000.0 * G;
    msg.linear_acceleration.z = az / 1000.0 * G;
    msg.linear_acceleration_covariance = {0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04};

    imu_pub_->publish(msg);
  }

  void publish_battery()
  {
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    double v;
    msg.voltage = field("v", v) ? static_cast<float>(v) : NAN;
    msg.present = msg.voltage > 5.0;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    battery_pub_->publish(msg);
  }

  // Afstelling (live aan te passen)
  double track_width_, speed_at_max_cmd_, max_wheel_cmd_;
  double min_wheel_cmd_, min_turn_wheel_cmd_, min_spin_wheel_cmd_;
  double spin_gain_, curve_gain_, cmd_timeout_;
  bool yaw_loop_;
  double yaw_kp_, yaw_ki_, yaw_i_max_;
  std::string imu_frame_;

  // Draairegeling (ROS-thread)
  double yaw_i_ = 0.0;              // integraal van de draaicorrectie (rad/s)
  std::optional<double> yaw_last_;  // tijd van het vorige commando
  std::optional<double> last_cmd_time_;

  // Gedeeld met de leesthread
  std::atomic<double> last_motion_{0.0};
  std::atomic<double> gyro_z_{0.0};     // gemeten draaisnelheid (rad/s, licht gefilterd)
  std::atomic<double> gyro_time_{0.0};

  // Alleen in de leesthread
  std::array<double, 3> gyro_bias_{0.0, 0.0, 0.0};  // °/s
  int gyro_samples_ = 0;
  double gyro_bias_alpha_;
  std::map<std::string, double> fields_;

  int fd_ = -1;
  std::mutex write_mutex_;
  std::atomic<bool> running_{true};
  std::atomic<bool> failed_{false};
  std::thread reader_;

  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr imu_timer_, battery_timer_, timeout_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<Esp32Bridge>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("esp32_bridge"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
