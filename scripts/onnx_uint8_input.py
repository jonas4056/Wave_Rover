"""Zet voor een YOLO-ONNX-model een ingang met ruwe RGB-bytes (1 x H x W x 3, uint8).

De omzetting naar float (/255) en naar kanalen-eerst (NCHW) gebeurt dan op de GPU in
plaats van met numpy op de CPU.
    python3 onnx_uint8_input.py yolo26n.onnx yolo26n_rgb8.onnx
"""
import sys

import onnx
from onnx import TensorProto, helper

src, dst = sys.argv[1], sys.argv[2]
model = onnx.load(src)
graph = model.graph
old = graph.input[0]
_, channels, height, width = [d.dim_value for d in old.type.tensor_type.shape.dim]

new = helper.make_tensor_value_info('rgb8', TensorProto.UINT8, [1, height, width, channels])
scale = helper.make_tensor('inv255', TensorProto.FLOAT, [], [1 / 255.0])
nodes = [
    helper.make_node('Cast', ['rgb8'], ['rgb_float'], to=TensorProto.FLOAT),
    helper.make_node('Mul', ['rgb_float', 'inv255'], ['rgb_scaled']),
    helper.make_node('Transpose', ['rgb_scaled'], [old.name], perm=[0, 3, 1, 2]),
]
graph.initializer.append(scale)
graph.input.remove(old)
graph.input.insert(0, new)
for node in reversed(nodes):
    graph.node.insert(0, node)
onnx.checker.check_model(model)
onnx.save(model, dst)
print(f'{dst}: ingang rgb8 1x{height}x{width}x{channels} uint8')
