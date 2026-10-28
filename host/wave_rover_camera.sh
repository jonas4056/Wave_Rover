#!/bin/bash
# Draait op de Jetson (buiten de container), gestart door wave-rover-camera.path.
# Zolang ~/wave_rover/run/camera_on bestaat, stuurt de CSI-camera hardware-gecodeerde
# JPEG-beelden naar 127.0.0.1:5600 (TCP), waar camera_bridge in de container ze ophaalt.
# Wordt het bestand verwijderd (module uit), dan stopt de camera.
REQUEST=/home/jonas/wave_rover/run/camera_on
PORT=5600

gst-launch-1.0 -q nvarguscamerasrc \
    ! "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1" \
    ! videorate drop-only=true max-rate=15 \
    ! nvvidconv ! "video/x-raw(memory:NVMM),format=I420" \
    ! nvjpegenc quality=75 \
    ! tcpserversink host=127.0.0.1 port=$PORT sync=false &
GST=$!

while [ -e "$REQUEST" ] && kill -0 "$GST" 2>/dev/null; do
    sleep 1
done
kill -INT "$GST" 2>/dev/null
wait "$GST"
