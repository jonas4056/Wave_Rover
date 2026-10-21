#!/bin/bash
# Overzicht van de robot in één keer (vanaf de pc): batterijen, modules, schijf en opstarts.
set -e

echo "== Batterijen"
ssh jetson 'cd ~/wave_rover && docker compose exec -T ros /entrypoint.sh python3 - <<"PY"
import rclpy, time
from sensor_msgs.msg import BatteryState
rclpy.init(); n = rclpy.create_node("status_battery"); got = {}
for t in ("ups/battery", "chassis/battery"):
    n.create_subscription(BatteryState, t, lambda m, t=t: got.__setitem__(t, m), 10)
end = time.time() + 4
while time.time() < end and len(got) < 2:
    rclpy.spin_once(n, timeout_sec=0.1)
status = {1: "laadt", 2: "ontlaadt", 3: "laadt niet", 4: "vol"}
for t, name in (("ups/battery", "Jetson "), ("chassis/battery", "Chassis")):
    m = got.get(t)
    if m is None:
        print(f"  {name}: geen data"); continue
    extra = f", {m.current:+.2f} A, {status.get(m.power_supply_status, "?")}" if t == "ups/battery" else ""
    print(f"  {name}: {m.voltage:5.2f} V  {m.percentage * 100:3.0f}%{extra}")
PY'

echo "== Modules"
ssh jetson 'cd ~/wave_rover && docker compose exec -T ros /entrypoint.sh bash -c "timeout 6 ros2 topic echo /modules/status --once --field data 2>/dev/null"' \
    | python3 -c 'import json, sys
for name, module in json.loads(sys.stdin.readline()).items():
    print("  %-12s %s" % (name, module["state"]))' \
    || echo "  geen data"

echo "== Schijf (NVMe/I/O-fouten sinds opstart)"
ssh jetson 'n=$(sudo dmesg | grep -ciE "nvme.*(timeout|reset|error)|I/O error|controller is down"); echo "  $n fouten; uptime: $(uptime -p)"'

echo "== Laatste opstarts"
"$(dirname "$0")/boot_history.sh" 4 | sed 's/^/  /'
