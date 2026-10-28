#!/bin/bash
# Kopieert de repo van de pc naar ~/wave_rover op de Jetson.
# build/, install/, log/, maps/ en run/ op de Jetson blijven staan.
# host/ bevat scripts die buiten de container draaien (zie scripts/install_camera.sh).
set -e

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

rsync -az --delete \
    --exclude .git \
    --exclude build --exclude install --exclude log --exclude maps --exclude run \
    "$REPO_DIR"/ jetson:wave_rover/
