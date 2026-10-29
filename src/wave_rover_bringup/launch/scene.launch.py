"""Scènebeschrijving (module scene): beeldtaalmodel op de GPU + scene_describer.

llama-server (llama.cpp met CUDA, in het image gebouwd) laadt het model in het geheugen zolang
de module aan staat (±2 GB voor Qwen3-VL-2B) en rekent alleen als er iets gevraagd wordt.
Modellen in ~/wave_rover/models/vlm en de stem in ~/wave_rover/models/piper
(scripts/download_scene_models.sh). Ander model: `model:=gemma3-4b` (beter Nederlands,
trager, ±4 GB).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VLM_DIR = '/home/ubuntu/wave_rover/models/vlm'
MODELS = {
    'qwen3vl-2b': ('Qwen3VL-2B-Instruct-Q4_K_M.gguf', 'mmproj-Qwen3VL-2B-Instruct-Q8_0.gguf'),
    'gemma3-4b': ('gemma-3-4b-it-Q4_K_M.gguf', 'mmproj-model-f16.gguf'),
}


def server(context):
    model, mmproj = MODELS[LaunchConfiguration('model').perform(context)]
    return [ExecuteProcess(
        cmd=['llama-server', '-m', f'{VLM_DIR}/{model}', '--mmproj', f'{VLM_DIR}/{mmproj}',
             '--n-gpu-layers', '99', '--ctx-size', '4096', '--parallel', '1',
             '--host', '127.0.0.1', '--port', '8090', '--no-webui'],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='qwen3vl-2b', choices=list(MODELS)),
        OpaqueFunction(function=server),
        Node(
            package='wave_rover_vision',
            executable='scene_describer',
            output='screen',
        ),
    ])
