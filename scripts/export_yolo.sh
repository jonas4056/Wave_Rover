#!/bin/bash
# Exporteert YOLO26n naar ONNX (640x384, NMS in het model, ingang ruwe RGB-bytes) en zet
# het op de Jetson.
# Draait op de pc (PyTorch hoeft niet op de Jetson). Daarna bouwt de module detection
# bij de eerste start zelf het TensorRT-engine.
#   bash scripts/export_yolo.sh
set -euo pipefail
WORK=${WORK:-/tmp/wave_rover_yolo}
mkdir -p "$WORK" && cd "$WORK"
[ -d venv ] || python3 -m venv venv
venv/bin/pip install -q --index-url https://download.pytorch.org/whl/cpu torch torchvision
venv/bin/pip install -q ultralytics onnx onnxslim
venv/bin/yolo export model=yolo26n.pt format=onnx imgsz=384,640 opset=17 simplify=True nms=False
# Ruwe RGB-bytes als ingang: de omzetting naar float gebeurt op de GPU i.p.v. de CPU.
venv/bin/python "$(dirname "$(readlink -f "$0")")/onnx_uint8_input.py" yolo26n.onnx yolo26n_rgb8.onnx
ssh jetson 'mkdir -p ~/wave_rover/models'
scp yolo26n_rgb8.onnx jetson:wave_rover/models/
