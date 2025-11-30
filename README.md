# Wave Rover

ROS 2 workspace for the Wave Rover project — an autonomous / manually RC-driven robot platform using:
- **Jetson Orin Nano**
- **ESP32 motor controller**
- **Crossfire RC (TBS Nano RX + TX16S transmitter)**
- **RPLidar**
- **RGB camera**
- **Yahboom speech board**

This workspace contains **all ROS 2 packages** needed to run the robot and the boot-time startup script/service.

---

## 📦 Packages inside `src/`

### Core Control
- `esp32_serial_bridge`  
  Subscribes to `/cmd_vel`, converts Twist messages to JSON motor commands, and sends them to the ESP32 via serial.

- `simple_joy_to_cmd`  
  Subscribes to `/joy` (Crossfire RC bridge output) and publishes `/cmd_vel` for the ESP32 motor controller.
  - Left stick: forward/backward → `linear.x`
  - Right stick: left/right turn → `angular.z`

- `rc_mapping_bringup`  
  Launch package for RC control, starting:
    1. `crsf_receiver_node` → `/rc/channels`
    2. `crsf_to_joy_node` → `/joy`
    3. `simple_joy_to_cmd_node` → `/cmd_vel`

### Sensors
- `argus_camera_publisher` — RGB camera node for video streaming or vision tasks.
- `yahboom_bridge` — Interface to the Yahboom speech board (`/yahboom_command` topic).
- `ros2_crsf_receiver` — Reads RC stick data directly from Crossfire receiver via UART.

---

## 🚀 Installation & Setup

### Clone the workspace
```bash
git clone https://github.com/jonas4056/Wave_Rover.git ~/ros2_ws
cd ~/ros2_ws
