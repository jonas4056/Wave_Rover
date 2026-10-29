#!/usr/bin/env python3
"""Vergelijkt twee versies van esp32_bridge (Python en C++) met een nagebootste ESP32.

Beide krijgen via een pseudo-terminal exact dezelfde antwoorden van het "board" en dezelfde
/cmd_vel-commando's; het script vergelijkt wat ze naar het board sturen en publiceren:
opstartcommando's, IMU (nulpunt, eenheden, quaternion), batterij, wielwaarden voor een raster
van snelheden (tekst moet exact gelijk zijn), de cmd_timeout, live parameters, de
gyroregeling (met een robot die onderstuurt), stoppen bij afsluiten en het CPU-gebruik.

Draait in een eigen ROS-domein (77, alleen localhost): de echte robot merkt er niets van.
Op de Jetson in de container (de seriële poort van de echte bridge blijft onaangeroerd):

    docker compose exec ros python3 scripts/compare_esp32_bridge.py

Of met eigen commando's:  --python "python3 -m ..." --cpp "/pad/naar/esp32_bridge"
"""
import argparse
import json
import math
import os
import pty
import select
import shlex
import signal
import subprocess
import threading
import time
import tty

os.environ['ROS_DOMAIN_ID'] = '77'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402  (pas na het kiezen van het domein)
from geometry_msgs.msg import TwistStamped  # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import BatteryState, Imu  # noqa: E402

BIAS = (-12.4, -21.6, 22.5)  # °/s, zoals het echte board op 29-10-2026
UNDERSTEER = 0.6             # de nagebootste robot draait 60% van wat de wielen vragen
GRID_VX = (-0.3, -0.1, 0.0, 0.004, 0.05, 0.1, 0.28, 0.5, 1.5)
GRID_WZ = (-3.0, -1.2, -0.5, -0.1, 0.0, 0.002, 0.1, 0.5, 1.2)


class FakeEsp32:
    """Antwoordt op T:126 (IMU) en T:130 (chassis) en houdt bij wat de bridge stuurt."""

    def __init__(self):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.port = os.ttyname(self.slave)
        self.lines = []        # (tijd, regel) van de bridge
        self.wheels = (0.0, 0.0)
        self.seq = 0
        self.running = True
        self.lock = threading.Lock()
        threading.Thread(target=self.loop, daemon=True).start()

    def write(self, text):
        os.write(self.master, text.encode())

    def imu_line(self):
        self.seq += 1
        left, right = self.wheels
        # Wielwaarde -> m/s (1,25 m/s bij 0,5), wielafstand 0,3 m, en de robot onderstuurt.
        rate = math.degrees(UNDERSTEER * (right - left) * 2.5 / 0.3)
        noise = [1.5 * math.sin(self.seq * k) for k in (1.7, 2.3, 3.1)]  # vast patroon
        g = [BIAS[i] + noise[i] + (rate if i == 2 else 0.0) for i in range(3)]
        data = {'T': 1002, 'r': 1.5, 'p': -2.0, 'y': 37.0, 'ax': 12, 'ay': -20, 'az': 1003,
                'gx': round(g[0], 2), 'gy': round(g[1], 2), 'gz': round(g[2], 2),
                'mx': 120, 'my': -40, 'mz': 300, 'temp': 31.5}
        return json.dumps(data, separators=(',', ':')) + '\r\n'

    def loop(self):
        # Rommel bij het opstarten, zoals een board dat net reset.
        self.write('ESP32 booting...\r\n{"T":1002,"r":}\r\n{"T":1005,"msg":"hallo, test"}\r\n')
        buffer = b''
        while self.running:
            ready, _, _ = select.select([self.master], [], [], 0.05)
            if not ready:
                continue
            try:
                buffer += os.read(self.master, 4096)
            except OSError:
                return
            while b'\n' in buffer:
                raw, buffer = buffer.split(b'\n', 1)
                line = raw.decode(errors='replace').strip()
                with self.lock:
                    self.lines.append((time.monotonic(), line))
                try:
                    cmd = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if cmd.get('T') == 126:
                    text = self.imu_line()
                    if self.seq % 7 == 0:  # af en toe een regel in twee stukken
                        self.write(text[:15])
                        time.sleep(0.003)
                        text = text[15:]
                    self.write(text)
                elif cmd.get('T') == 130:
                    self.write('{"T":1001,"L":0,"R":0,"r":1.5,"p":-2.0,"y":37.0,"temp":31.2,'
                               '"v":12.13}\r\n')
                elif cmd.get('T') == 1:
                    self.wheels = (float(cmd['L']), float(cmd['R']))

    def wheel_lines(self, since=0.0):
        with self.lock:
            return [(t, line) for t, line in self.lines if line.startswith('{"T":1,') and t > since]

    def close(self):
        self.running = False
        time.sleep(0.1)
        os.close(self.master)
        os.close(self.slave)


def cpu_seconds(pid):
    """CPU-tijd (s) van een proces en zijn kinderen (ros2 run start de node als kind)."""
    total = 0.0
    tick = os.sysconf('SC_CLK_TCK')
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/stat') as f:
                fields = f.read().rsplit(')', 1)[1].split()
        except OSError:
            continue
        if int(entry) == pid or int(fields[1]) == pid:
            total += (int(fields[11]) + int(fields[12])) / tick
    return total


def run(name, command):
    """Eén versie testen; geeft een dict met alles wat vergeleken wordt."""
    fake = FakeEsp32()
    proc = subprocess.Popen(shlex.split(command) + ['--ros-args', '-p', f'port:={fake.port}'],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    node = rclpy.create_node(f'compare_{name}')
    imu, battery = [], []
    node.create_subscription(Imu, 'imu/data_raw', imu.append, qos_profile_sensor_data)
    node.create_subscription(BatteryState, 'battery', battery.append, 10)
    cmd_pub = node.create_publisher(TwistStamped, 'cmd_vel', 10)
    params = node.create_client(SetParameters, '/esp32_bridge/set_parameters')
    result = {}

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def wait_for(check, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if check():
                return True
            rclpy.spin_once(node, timeout_sec=0.01)
        return False

    def set_param(pname, value):
        p = Parameter(name=pname)
        if isinstance(value, bool):
            p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        elif isinstance(value, int):
            p.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
        else:
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
        future = params.call_async(SetParameters.Request(parameters=[p]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        return bool(future.result() and future.result().results[0].successful)

    def command(vx, wz):
        """Eén /cmd_vel sturen en de wielregel teruggeven die de bridge daarop stuurt."""
        before = time.monotonic()
        msg = TwistStamped()
        msg.twist.linear.x, msg.twist.angular.z = vx, wz
        cmd_pub.publish(msg)
        ok = wait_for(lambda: fake.wheel_lines(before), 0.5)
        return fake.wheel_lines(before)[0][1] if ok else None

    try:
        wait_for(lambda: len(fake.lines) >= 2, 10)
        result['opstart'] = [line for _, line in fake.lines[:2]]

        # IMU: wacht op het nulpunt (2 s stilstand na de eerste seconde), dan 40 berichten.
        wait_for(lambda: len(imu) >= 40, 8)
        result['imu_na_s'] = round(len(imu) / 20.0, 1)
        last = imu[-40:]
        mean = lambda xs: sum(xs) / len(xs)  # noqa: E731
        result['imu'] = {
            'gyro_rest_deg_s': [round(math.degrees(mean([getattr(m.angular_velocity, a) for m in last])), 2)
                                for a in 'xyz'],
            'acc': [round(getattr(last[-1].linear_acceleration, a), 4) for a in 'xyz'],
            'quat': [round(getattr(last[-1].orientation, a), 6) for a in 'xyzw'],
            'cov': [list(last[-1].orientation_covariance), list(last[-1].angular_velocity_covariance),
                    list(last[-1].linear_acceleration_covariance)],
            'frame': last[-1].header.frame_id,
        }
        wait_for(lambda: battery, 3)
        b = battery[-1] if battery else None
        result['batterij'] = (round(b.voltage, 2), b.present, b.power_supply_technology) if b else None

        # Wielwaarden zonder gyroregeling: tekst moet exact gelijk zijn.
        params.wait_for_service(timeout_sec=3)
        result['param_yaw_loop'] = set_param('yaw_loop', False)
        result['raster'] = [[vx, wz, command(vx, wz)] for vx in GRID_VX for wz in GRID_WZ]

        # Stoppen na cmd_timeout (0,5 s; de controle loopt elke 0,1 s).
        last_cmd = time.monotonic()
        wait_for(lambda: any(line == '{"T":1,"L":0,"R":0}' for _, line in fake.wheel_lines(last_cmd)), 2)
        stops = [t for t, line in fake.wheel_lines(last_cmd) if line == '{"T":1,"L":0,"R":0}']
        result['timeout_s'] = round(stops[0] - last_cmd, 2) if stops else None

        # Live parameters: andere spin_gain, en een verkeerd type moet geweigerd worden.
        result['param_spin_gain'] = set_param('spin_gain', 2.0)
        result['met_spin_gain_2'] = command(0.0, 1.0)
        result['param_fout_type'] = set_param('min_spin_wheel_cmd', 1)
        set_param('spin_gain', 1.6)

        # Gyroregeling: 4 s bocht aan 20 Hz; de robot onderstuurt, de regelaar moet bijsturen.
        result['param_yaw_loop_aan'] = set_param('yaw_loop', True)
        spin(1.5)  # vorige test helemaal uitgedraaid, zodat beide hetzelfde vertrekpunt hebben
        start = time.monotonic()
        msg = TwistStamped()
        msg.twist.linear.x, msg.twist.angular.z = 0.2, 0.5
        while time.monotonic() - start < 4.0:
            cmd_pub.publish(msg)
            spin(0.05)
        lines = [json.loads(line) for _, line in fake.wheel_lines(start)]
        tail = lines[-20:]
        result['regeling_eerste'] = lines[0] if lines else None
        result['regeling_eind'] = [round(mean([c['L'] for c in tail]), 3),
                                   round(mean([c['R'] for c in tail]), 3)] if tail else None
        meas = imu[-20:]
        result['regeling_gyro_deg_s'] = round(math.degrees(mean([m.angular_velocity.z for m in meas])), 1)
        spin(1.0)

        # CPU in rust (5 s, alleen IMU en batterij opvragen).
        cpu0, t0 = cpu_seconds(proc.pid), time.monotonic()
        spin(5.0)
        result['cpu_rust_pct'] = round(100 * (cpu_seconds(proc.pid) - cpu0) / (time.monotonic() - t0), 1)

        # Afsluiten: motoren moeten een laatste stop krijgen.
        before = time.monotonic()
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=10)
        time.sleep(0.2)
        result['afsluiten'] = [line for _, line in fake.wheel_lines(before)]
    finally:
        if proc.poll() is None:
            proc.kill()
        output = proc.communicate()[0]
        node.destroy_node()
        fake.close()
    result['log'] = [line for line in output.splitlines() if 'Gyronulpunt' in line or 'rror' in line]
    return result


def compare(a, b):
    """Afwijkingen tussen de twee resultaten; lege lijst = gelijk."""
    problems = []
    exact = ('opstart', 'batterij', 'param_yaw_loop', 'param_spin_gain', 'met_spin_gain_2',
             'param_fout_type', 'param_yaw_loop_aan', 'afsluiten')
    for key in exact:
        if a.get(key) != b.get(key):
            problems.append(f'{key}: {a.get(key)} <> {b.get(key)}')
    for (vx, wz, la), (_, _, lb) in zip(a['raster'], b['raster']):
        if la != lb:
            problems.append(f'raster vx={vx} wz={wz}: {la} <> {lb}')
    ia, ib = a['imu'], b['imu']
    for key in ('acc', 'quat', 'cov', 'frame'):
        if ia[key] != ib[key]:
            problems.append(f'imu {key}: {ia[key]} <> {ib[key]}')
    if any(abs(x - y) > 0.3 for x, y in zip(ia['gyro_rest_deg_s'], ib['gyro_rest_deg_s'])):
        problems.append(f'imu gyro in rust: {ia["gyro_rest_deg_s"]} <> {ib["gyro_rest_deg_s"]}')
    if None in (a['timeout_s'], b['timeout_s']) or abs(a['timeout_s'] - b['timeout_s']) > 0.15:
        problems.append(f'timeout: {a["timeout_s"]} <> {b["timeout_s"]}')
    ea, eb = a['regeling_eind'], b['regeling_eind']
    if ea is None or eb is None or any(abs(x - y) > 0.01 for x, y in zip(ea, eb)):
        problems.append(f'gyroregeling eindwaarde: {ea} <> {eb}')
    return problems


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--python', default='ros2 run wave_rover_base esp32_bridge')
    parser.add_argument('--cpp', default='ros2 run wave_rover_base_cpp esp32_bridge')
    args = parser.parse_args()
    rclpy.init()
    results = {}
    for name, command in (('python', args.python), ('cpp', args.cpp)):
        print(f'== {name}: {command}', flush=True)
        results[name] = run(name, command)
        r = results[name]
        print(json.dumps({k: v for k, v in r.items() if k != 'raster'}, ensure_ascii=False))
    rclpy.try_shutdown()
    problems = compare(results['python'], results['cpp'])
    print(f'\nRaster: {len(results["cpp"]["raster"])} commando\'s vergeleken')
    print(f'CPU in rust: python {results["python"]["cpu_rust_pct"]}%, cpp {results["cpp"]["cpu_rust_pct"]}% van een kern')
    if problems:
        print(f'VERSCHILLEN ({len(problems)}):')
        for p in problems:
            print('  -', p)
        raise SystemExit(1)
    print('GELIJK: geen verschillen gevonden')


if __name__ == '__main__':
    main()
