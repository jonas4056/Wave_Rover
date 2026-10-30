"""Scènebeschrijving (module scene): beeldtaalmodel op de GPU + scene_describer.

llama-server (llama.cpp met CUDA, in het image gebouwd) laadt het model in het geheugen zolang
de module aan staat en rekent alleen als er iets gevraagd wordt. Standaard Qwen3-VL-4B (±3,4 GB):
vergeleken op 30-10-2026 (scripts/compare_vlm.py) spreekt het duidelijk beter Nederlands dan
de 2B ("metalen prullenbak" i.p.v. "wastafel"), 2-3 s per antwoord. `model:=qwen3vl-2b`
spaart ±0,8 GB en vond kleine voorwerpen soms beter. Modellen in ~/wave_rover/models/vlm,
de stem in ~/wave_rover/models/piper (scripts/download_scene_models.sh).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VLM_DIR = '/home/ubuntu/wave_rover/models/vlm'
MODELS = {
    'qwen3vl-2b': ('Qwen3VL-2B-Instruct-Q4_K_M.gguf', 'mmproj-Qwen3VL-2B-Instruct-Q8_0.gguf'),
    'qwen3vl-4b': ('Qwen3VL-4B-Instruct-Q4_K_M.gguf', 'mmproj-Qwen3VL-4B-Instruct-Q8_0.gguf'),
    # Getest 29-10-2026: beter Nederlands, maar ±6 s per foto, ±3,5 GB en geen kaders.
    'gemma3-4b': ('gemma-3-4b-it-Q4_K_M.gguf', 'mmproj-model-f16.gguf'),
}


def server(context):
    model, mmproj = MODELS[LaunchConfiguration('model').perform(context)]
    return [ExecuteProcess(
        cmd=['llama-server', '-m', f'{VLM_DIR}/{model}', '--mmproj', f'{VLM_DIR}/{mmproj}',
             # 2048 tokens: foto (±240) + vraag + antwoord passen ruim; spaart ±0,3 GB t.o.v. 4096.
             '--n-gpu-layers', '99', '--ctx-size', '2048', '--parallel', '1',
             '--host', '127.0.0.1', '--port', '8090', '--no-webui'],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='qwen3vl-4b', choices=list(MODELS)),
        OpaqueFunction(function=server),
        Node(
            package='wave_rover_vision',
            executable='scene_describer',
            output='screen',
        ),
    ])
