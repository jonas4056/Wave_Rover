#!/bin/bash
# Installeert de udev-regels op de Jetson (vanaf de pc uitvoeren).
set -e

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

scp "$REPO_DIR"/udev/99-wave-rover.rules jetson:/tmp/
ssh jetson 'sudo install -m 0644 /tmp/99-wave-rover.rules /etc/udev/rules.d/ \
    && sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty'
