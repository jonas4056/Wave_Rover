#!/bin/bash
# Bouwt en installeert het CH341/CH340-stuurprogramma (USB-serieel van het Yahboom-spraakbord).
# De JetPack 7-kernel (6.8-tegra) levert ch341 niet mee. We compileren ch341.c uit de
# officiële Linux-broncode van dezelfde kernelversie tegen de geïnstalleerde headers.
#
# Op de Jetson uitvoeren; na een kernelupdate opnieuw uitvoeren.
set -e

KERNEL_TAG=v6.8
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

cd "$WORK"
curl -fsSL -o ch341.c \
    "https://raw.githubusercontent.com/torvalds/linux/${KERNEL_TAG}/drivers/usb/serial/ch341.c"
echo 'obj-m := ch341.o' > Makefile
make -C "/lib/modules/$(uname -r)/build" M="$WORK" modules

sudo install -D -m 0644 ch341.ko "/lib/modules/$(uname -r)/updates/ch341.ko"
sudo depmod -a
sudo modprobe ch341
echo "ch341 geïnstalleerd voor kernel $(uname -r)"
