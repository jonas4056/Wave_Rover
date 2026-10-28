#!/bin/bash
# Geeft Docker-containers toegang tot de GPU van de Jetson via CDI.
# Opnieuw uitvoeren na een JetPack-/L4T-update (de driverbestanden veranderen dan).
# Op de Jetson: bash ~/wave_rover/scripts/install_gpu.sh
set -euo pipefail
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
nvidia-ctk cdi list
