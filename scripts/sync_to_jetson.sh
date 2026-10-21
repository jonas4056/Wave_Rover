#!/bin/bash
# Kopieert de repo van de pc naar ~/wave_rover op de Jetson.
# build/, install/ en log/ op de Jetson blijven staan.
set -e

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

rsync -az --delete \
    --exclude .git \
    --exclude build --exclude install --exclude log \
    "$REPO_DIR"/ jetson:wave_rover/
