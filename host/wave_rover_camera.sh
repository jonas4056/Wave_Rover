#!/bin/bash
# Draait op de Jetson (buiten de container), gestart door wave-rover-camera.path.
# Zolang ~/wave_rover/run/camera_on bestaat, stuurt de CSI-camera hardware-gecodeerde
# JPEG-beelden over TCP, waar camera_bridge in de container ze ophaalt:
# - 127.0.0.1:5600: 1280x720 (ROS-topics, herkenning)
# - 127.0.0.1:5601: 640x360 voorbeeld voor het dashboard (~25 kB i.p.v. ~110 kB per beeld,
#   anders haalt een gsm over wifi maar 5-8 beelden/s en schokt het beeld)
# Schalen (VIC) en coderen (NVJPG) gebeuren in hardware. De sensor draait zelf op de
# gevraagde framerate (nvargus-daemon: ~10% van één kern bij 15 fps, ~20% bij 30 fps).
# Wordt het bestand verwijderd (module uit), dan stopt de camera.
REQUEST=/home/jonas/wave_rover/run/camera_on
PORT=5600
PREVIEW_PORT=5601
# Framerate komt uit het verzoekbestand ("fps=30"), gezet door camera_bridge.
FPS=$(grep -oE 'fps=[0-9]+' "$REQUEST" 2>/dev/null | cut -d= -f2)
FPS=${FPS:-30}

gst-launch-1.0 -q nvarguscamerasrc \
    ! "video/x-raw(memory:NVMM),width=1280,height=720,framerate=$FPS/1" ! tee name=t \
    t. ! queue max-size-buffers=2 leaky=downstream \
       ! nvvidconv ! "video/x-raw(memory:NVMM),format=I420" \
       ! nvjpegenc quality=75 \
       ! tcpserversink host=127.0.0.1 port=$PORT sync=false \
    t. ! queue max-size-buffers=2 leaky=downstream \
       ! nvvidconv ! "video/x-raw(memory:NVMM),width=640,height=360,format=I420" \
       ! nvjpegenc quality=70 \
       ! tcpserversink host=127.0.0.1 port=$PREVIEW_PORT sync=false &
GST=$!

while [ -e "$REQUEST" ] && kill -0 "$GST" 2>/dev/null; do
    sleep 1
done
kill -INT "$GST" 2>/dev/null
wait "$GST"
