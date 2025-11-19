#!/bin/bash
# ==========================================
# WaveRover Auto-Start Script (Stable+Monitored)
# ==========================================
# Starts ESP32 bridge + LiDAR + CSI Camera + Yahboom nodes on Jetson boot
# Ensures devices exist, logs everything, verifies ROS2 nodes
# ==========================================

LOGFILE="/home/jonas-nano/wave_rover_startup.log"
ESP32_DEV="/dev/esp32"
LIDAR_DEV="/dev/lidar"
YAHBOOM_DEV="/dev/ttyCH341USB0"

echo ""
echo "[$(date)] 🚀 WaveRover startup script launched" | tee -a "$LOGFILE"
echo "[$(date)] Logging to: $LOGFILE" | tee -a "$LOGFILE"
echo "=============================================" | tee -a "$LOGFILE"

# --- Wait for devices to appear ---
for i in {1..15}; do
    if [ -e "$ESP32_DEV" ] && [ -e "$LIDAR_DEV" ]; then
        echo "[$(date)] ✅ Devices detected: $ESP32_DEV and $LIDAR_DEV" | tee -a "$LOGFILE"
        break
    fi
    echo "[$(date)] ⏳ Waiting for devices to connect... ($i/15)" | tee -a "$LOGFILE"
    sleep 2
done

# Abort if devices are still missing
if [ ! -e "$ESP32_DEV" ] || [ ! -e "$LIDAR_DEV" ]; then
    echo "[$(date)] ❌ Error: Devices not found after 30s. Aborting startup." | tee -a "$LOGFILE"
    exit 1
fi

# --- Source ROS environments ---
source /opt/ros/humble/setup.bash
source /home/jonas-nano/ros2_ws/install/setup.bash

# --- Give ROS a few seconds to settle ---
sleep 5

# --- Start Yahboom Serial Bridge ---
echo "[$(date)] ▶ Starting Yahboom Bridge on $YAHBOOM_DEV..." | tee -a "$LOGFILE"
nohup ros2 run yahboom_bridge yahboom_serial_bridge --ros-args -p port:=$YAHBOOM_DEV >> "$LOGFILE" 2>&1 &

# --- Wait until the serial bridge is active ---
for i in {1..10}; do
    if lsof "$YAHBOOM_DEV" >/dev/null 2>&1; then
        echo "[$(date)] ✅ Yahboom serial bridge active." | tee -a "$LOGFILE"
        break
    fi
    echo "[$(date)] ⏳ Waiting for Yahboom serial bridge to initialize... ($i/10)" | tee -a "$LOGFILE"
    sleep 1
done

# --- Start Yahboom Command Handler ---
echo "[$(date)] ▶ Starting Yahboom Command Handler..." | tee -a "$LOGFILE"
nohup ros2 run yahboom_bridge robot_command_handler >> "$LOGFILE" 2>&1 &# --- Start ESP32 Bridge ---
echo "[$(date)] ▶ Starting ESP32 bridge..." | tee -a "$LOGFILE"
nohup ros2 run esp32_serial_bridge esp32_serial_node --ros-args -p port:=$ESP32_DEV >> "$LOGFILE" 2>&1 &

# --- Start LiDAR ---
echo "[$(date)] ▶ Starting RPLIDAR..." | tee -a "$LOGFILE"
nohup ros2 launch rplidar_ros rplidar_a3_launch.py serial_port:=$LIDAR_DEV serial_baudrate:=460800 scan_mode:=Standard >> "$LOGFILE" 2>&1 &

# --- Start Argus CSI camera ---
echo "[$(date)] ▶ Starting Argus CSI camera publisher..." | tee -a "$LOGFILE"
nohup ros2 run argus_camera_publisher argus_camera_node >> "$LOGFILE" 2>&1 &

# Start TF broadcaster for LiDAR
ros2 launch wave_rover_bringup tf_laser.launch.py &

# Start SLAM Toolbox
ros2 launch wave_rover_bringup slam.launch.py &

# --- Wait for nodes to come online ---
echo "[$(date)] ⏳ Checking ROS2 nodes status..." | tee -a "$LOGFILE"
sleep 10

ROS_NODES=$(ros2 node list 2>/dev/null)
echo "$ROS_NODES" | tee -a "$LOGFILE"

if echo "$ROS_NODES" | grep -Eq "esp32_serial_bridge|wave_rover_bridge"; then
    echo "[$(date)] ✅ ESP32 Bridge Node running" | tee -a "$LOGFILE"
else
    echo "[$(date)] ❌ ESP32 Bridge Node NOT detected!" | tee -a "$LOGFILE"
fi

if echo "$ROS_NODES" | grep -q "rplidar_node"; then
    echo "[$(date)] ✅ LiDAR Node running" | tee -a "$LOGFILE"
else
    echo "[$(date)] ❌ LiDAR Node NOT detected!" | tee -a "$LOGFILE"
fi

if echo "$ROS_NODES" | grep -q "argus_camera_publisher"; then
    echo "[$(date)] ✅ Camera Node running" | tee -a "$LOGFILE"
else
    echo "[$(date)] ❌ Camera Node NOT detected!" | tee -a "$LOGFILE"
fi
if echo "$ROS_NODES" | grep -q "yahboom_bridge"; then
    echo "[$(date)] ✅ Yahboom Bridge Node running" | tee -a "$LOGFILE"
else
    echo "[$(date)] ❌ Yahboom Bridge Node NOT detected!" | tee -a "$LOGFILE"
fi

if echo "$ROS_NODES" | grep -q "robot_command_handler"; then
    echo "[$(date)] ✅ Yahboom Command Handler Node running" | tee -a "$LOGFILE"
else
    echo "[$(date)] ❌ Yahboom Command Handler Node NOT detected!" | tee -a "$LOGFILE"
fi
ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link lidar_link &
ros2 run tf2_ros static_transform_publisher 0.12 0 0.3 0 -0.1745 0 base_link camera_link &
ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 base_link imu_link &

echo "[$(date)] ✅ WaveRover startup complete. Monitoring logs in $LOGFILE" | tee -a "$LOGFILE"
echo "=============================================" | tee -a "$LOGFILE"

# --- Keep service alive indefinitely ---
while true; do
    sleep 60
done
