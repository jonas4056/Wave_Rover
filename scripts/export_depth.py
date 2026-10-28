"""Exporteert Depth Anything V2 Small naar ONNX voor de module floor_obstacles.

Ingang: ruwe RGB-bytes (1 x 294 x 518 x 3, uint8); omzetting naar float en de
ImageNet-normalisatie gebeuren in het model (op de GPU). Uitgang: relatieve inverse diepte
(1 x 294 x 518), groter = dichterbij. Licentie van het Small-model: Apache-2.0.
Draait op de pc (PyTorch hoeft niet op de Jetson); zie scripts/export_yolo.sh voor de venv.
    venv/bin/python export_depth.py depth_anything_v2_small_rgb8.onnx
"""
import sys

import torch
from transformers import AutoModelForDepthEstimation

HEIGHT, WIDTH = 294, 518  # veelvouden van 14 (patchgrootte), ongeveer 16:9


class Wrapper(torch.nn.Module):

    def __init__(self, model):
        super().__init__()
        self.model = model
        self.register_buffer('mean', torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1))
        self.register_buffer('std', torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1))

    def forward(self, rgb8):
        # Eerst naar float: TensorRT aanvaardt op uint8 enkel een Cast.
        x = rgb8.float().permute(0, 3, 1, 2) / 255.0
        return self.model(pixel_values=(x - self.mean) / self.std).predicted_depth


model = AutoModelForDepthEstimation.from_pretrained('depth-anything/Depth-Anything-V2-Small-hf')
wrapper = Wrapper(model.eval())
dummy = torch.zeros(1, HEIGHT, WIDTH, 3, dtype=torch.uint8)
torch.onnx.export(wrapper, dummy, sys.argv[1], input_names=['rgb8'], output_names=['depth'],
                  opset_version=17, dynamo=False)
print(f'{sys.argv[1]}: ingang 1x{HEIGHT}x{WIDTH}x3 uint8')
