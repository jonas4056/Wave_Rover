#!/bin/bash
# Installeert op de Jetson de systemd-units voor de camera (vanaf de pc uitvoeren).
#   ~/wave_rover/run/camera_on verschijnt -> wave-rover-camera.service -> host/wave_rover_camera.sh
set -e

ssh jetson 'bash -s' <<'REMOTE'
set -e
mkdir -p ~/wave_rover/run
chmod +x ~/wave_rover/host/wave_rover_camera.sh

sudo tee /etc/systemd/system/wave-rover-camera.path >/dev/null <<'UNIT'
[Unit]
Description=Wave Rover: camera starten op verzoek van de module camera

[Path]
PathExists=/home/jonas/wave_rover/run/camera_on

[Install]
WantedBy=multi-user.target
UNIT

sudo tee /etc/systemd/system/wave-rover-camera.service >/dev/null <<'UNIT'
[Unit]
Description=Wave Rover: CSI-camera -> hardware-JPEG -> 127.0.0.1:5600
After=nvargus-daemon.service

[Service]
User=jonas
ExecStart=/home/jonas/wave_rover/host/wave_rover_camera.sh
UNIT

sudo systemctl daemon-reload
sudo systemctl enable --now wave-rover-camera.path
systemctl is-active wave-rover-camera.path
REMOTE
