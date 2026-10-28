#!/bin/bash
# Draait op de Jetson (buiten de container), gestart door wave-rover-camera.path.
# Zolang ~/wave_rover/run/camera_on bestaat, stuurt de CSI-camera hardware-gecodeerde
# JPEG-beelden naar 127.0.0.1:5600 (TCP). De sensor draait zelf op de gevraagde framerate
# (nvargus-daemon: ~10% van één kern bij 15 fps, ~20% bij 30 fps)., waar camera_bridge in de container ze ophaalt.
# Wordt het bestand verwijderd (module uit), dan stopt de camera.
REQUEST=/home/jonas/wave_rover/run/camera_on
PORT=5600
# Framerate komt uit het verzoekbestand ("fps=30"), gezet door camera_bridge.
FPS=$(grep -oE 'fps=[0-9]+' "$REQUEST" 2>/dev/null | cut -d= -f2)
FPS=${FPS:-30}

gst-launch-1.0 -q nvarguscamerasrc \
    ! "video/x-raw(memory:NVMM),width=1280,height=720,framerate=$FPS/1" \
    ! nvvidconv ! "video/x-raw(memory:NVMM),format=I420" \
    ! nvjpegenc quality=75 \
    ! tcpserversink host=127.0.0.1 port=$PORT sync=false &
GST=$!

while [ -e "$REQUEST" ] && kill -0 "$GST" 2>/dev/null; do
    sleep 1
done
kill -INT "$GST" 2>/dev/null
wait "$GST"
