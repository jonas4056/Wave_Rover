"""Een TensorRT-model draaien op de GPU van de Jetson, zonder PyTorch of PyCUDA.

- Het engine-bestand wordt eenmalig gebouwd uit een ONNX-model met trtexec (FP16). Het
  hoort bij één TensorRT-versie, dus die zit in de bestandsnaam: na een update van
  TensorRT wordt het vanzelf opnieuw gebouwd.
- CPU en GPU delen op de Jetson hetzelfde geheugen. Met managed memory zien beide
  dezelfde buffers, dus er wordt niets heen en weer gekopieerd.
"""
import ctypes
import glob
import os
import subprocess

import numpy as np
import tensorrt as trt

TRTEXEC = '/usr/src/tensorrt/bin/trtexec'
CUDA_MEM_ATTACH_GLOBAL = 1
CUDA_DEVICE_SCHEDULE_BLOCKING_SYNC = 4


def _load_cudart():
    for name in ['libcudart.so.13', *glob.glob('/usr/local/cuda*/targets/*/lib/libcudart.so.*')]:
        try:
            return ctypes.CDLL(name)
        except OSError:
            continue
    raise OSError('libcudart niet gevonden')


def engine_path(onnx_path):
    return f'{os.path.splitext(onnx_path)[0]}_trt{trt.__version__}.engine'


def build_engine(onnx_path, log):
    """Bouwt het engine-bestand als het nog niet bestaat (duurt enkele minuten)."""
    path = engine_path(onnx_path)
    if os.path.exists(path):
        return path
    log(f'TensorRT-engine bouwen uit {os.path.basename(onnx_path)}; dit duurt enkele minuten')
    tmp = path + '.tmp'
    result = subprocess.run([TRTEXEC, f'--onnx={onnx_path}', f'--saveEngine={tmp}', '--fp16'],
                            capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError('trtexec mislukt:\n' + result.stdout[-2000:] + result.stderr[-2000:])
    os.replace(tmp, path)  # pas na succes: een half bestand wordt nooit gebruikt
    return path


class TrtModel:

    def __init__(self, engine_file):
        self.cudart = _load_cudart()
        # Wachten op de GPU zonder de CPU te laten pollen; moet vóór de eerste CUDA-aanroep.
        self.cudart.cudaSetDeviceFlags(CUDA_DEVICE_SCHEDULE_BLOCKING_SYNC)
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_file, 'rb') as f:
            self.engine = trt.Runtime(self.logger).deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.stream = ctypes.c_void_p()
        self._check(self.cudart.cudaStreamCreate(ctypes.byref(self.stream)), 'cudaStreamCreate')

        self.inputs, self.outputs, self._pointers = {}, {}, []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            dtype = np.dtype(trt.nptype(self.engine.get_tensor_dtype(name)))
            array = self._managed_array(shape, dtype)
            self.context.set_tensor_address(name, array.ctypes.data)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.inputs[name] = array
            else:
                self.outputs[name] = array

    def _check(self, err, what):
        if err != 0:
            raise RuntimeError(f'{what} mislukt (CUDA-fout {err})')

    def _managed_array(self, shape, dtype):
        nbytes = int(np.prod(shape)) * dtype.itemsize
        ptr = ctypes.c_void_p()
        self._check(self.cudart.cudaMallocManaged(ctypes.byref(ptr), ctypes.c_size_t(nbytes),
                                                  CUDA_MEM_ATTACH_GLOBAL), 'cudaMallocManaged')
        self._pointers.append(ptr)
        buf = (ctypes.c_byte * nbytes).from_address(ptr.value)
        return np.frombuffer(buf, dtype=dtype).reshape(shape)

    def run(self):
        """Voert het model uit op de huidige inhoud van `inputs`; resultaat in `outputs`."""
        if not self.context.execute_async_v3(self.stream.value):
            raise RuntimeError('TensorRT-uitvoering mislukt')
        self._check(self.cudart.cudaStreamSynchronize(self.stream), 'cudaStreamSynchronize')

    def close(self):
        self.cudart.cudaStreamSynchronize(self.stream)
        self.inputs, self.outputs = {}, {}
        self.context = None
        self.engine = None
        for ptr in self._pointers:
            self.cudart.cudaFree(ptr)
        self._pointers = []
