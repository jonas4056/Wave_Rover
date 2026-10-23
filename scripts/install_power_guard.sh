#!/bin/bash
# Installeert op de Jetson de systemd-units die de Jetson afsluiten als power_guard
# (in de container) een verzoekbestand aanmaakt. Vanaf de pc uitvoeren.
#
#   ~/wave_rover/run/shutdown_request verschijnt -> wave-rover-shutdown.service -> poweroff
set -e

ssh jetson 'bash -s' <<'EOF'
set -e
mkdir -p ~/wave_rover/run
rm -f ~/wave_rover/run/shutdown_request

sudo tee /etc/systemd/system/wave-rover-shutdown.path >/dev/null <<'UNIT'
[Unit]
Description=Wave Rover: afsluiten op verzoek van power_guard (batterij leeg)

[Path]
PathExists=/home/jonas/wave_rover/run/shutdown_request

[Install]
WantedBy=multi-user.target
UNIT

sudo tee /etc/systemd/system/wave-rover-shutdown.service >/dev/null <<'UNIT'
[Unit]
Description=Wave Rover: Jetson afsluiten (batterij leeg)

[Service]
Type=oneshot
ExecStartPre=/usr/bin/rm -f /home/jonas/wave_rover/run/shutdown_request
ExecStart=/usr/bin/systemctl poweroff
UNIT

sudo systemctl daemon-reload
sudo systemctl enable --now wave-rover-shutdown.path
systemctl is-active wave-rover-shutdown.path
EOF
