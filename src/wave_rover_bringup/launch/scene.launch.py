"""Praten met Wave en wat hij ziet (module scene): beeldtaalmodel op de GPU + assistant.

llama-server (llama.cpp met CUDA, in het image gebouwd) laadt het model in het geheugen zolang
de module aan staat en rekent alleen als er iets gevraagd wordt. Standaard (model:=auto) Qwen3-VL-4B
(±3,4 GB) als er genoeg geheugen vrij is, anders de 2B (±2,6 GB). De 4B:
vergeleken op 30-10-2026 (scripts/compare_vlm.py) spreekt het duidelijk beter Nederlands dan
de 2B ("metalen prullenbak" i.p.v. "wastafel"), 2-3 s per antwoord. `model:=qwen3vl-2b`
spaart ±0,8 GB en vond kleine voorwerpen soms beter. Modellen in ~/wave_rover/models/vlm,
de stem in ~/wave_rover/models/piper (scripts/download_scene_models.sh).
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VLM_DIR = '/home/ubuntu/wave_rover/models/vlm'
# model:=auto kiest de 4B alleen met zoveel vrij geheugen (MB) bij het starten. Op 30-10-2026
# met navigation, detection, floor_obstacles en de 4B tegelijk: OOM-killer stopte llama-server
# toen Piper wou spreken (4,1 GB), en de GPU gedeeld door drie modellen: 12 s per antwoord.
AUTO_4B_MIN_AVAILABLE = 4800
MODELS = {
    'qwen3vl-2b': ('Qwen3VL-2B-Instruct-Q4_K_M.gguf', 'mmproj-Qwen3VL-2B-Instruct-Q8_0.gguf'),
    'qwen3vl-4b': ('Qwen3VL-4B-Instruct-Q4_K_M.gguf', 'mmproj-Qwen3VL-4B-Instruct-Q8_0.gguf'),
    # Getest 29-10-2026: beter Nederlands, maar ±6 s per foto, ±3,5 GB en geen kaders.
    'gemma3-4b': ('gemma-3-4b-it-Q4_K_M.gguf', 'mmproj-model-f16.gguf'),
}


def available_mb():
    with open('/proc/meminfo') as f:
        for line in f:
            if line.startswith('MemAvailable'):
                return int(line.split()[1]) // 1024
    return 0


def server(context):
    name = LaunchConfiguration('model').perform(context)
    free = available_mb()
    if name == 'auto':
        name = 'qwen3vl-4b' if free >= AUTO_4B_MIN_AVAILABLE else 'qwen3vl-2b'
    model, mmproj = MODELS[name]
    llama = ExecuteProcess(
        cmd=['llama-server', '-m', f'{VLM_DIR}/{model}', '--mmproj', f'{VLM_DIR}/{mmproj}',
             # 2048 tokens: foto (±240) + vraag + antwoord passen ruim; spaart ±0,3 GB t.o.v. 4096.
             '--n-gpu-layers', '99', '--ctx-size', '2048', '--parallel', '1',
             # Geen promptcache in het RAM (standaard tot 8 GB!) en kleinere rekenblokken: het
             # geheugen blijft dan vast i.p.v. per vraag te groeien. Een foto is ±240 tokens.
             '--cache-ram', '0', '--batch-size', '512', '--ubatch-size', '256',
             '--host', '127.0.0.1', '--port', '8090', '--no-webui'],
        output='screen',
    )
    return [
        LogInfo(msg=f'Beeldtaalmodel {name} ({free} MB vrij)'),
        llama,
        # Valt het model weg (bv. geheugen vol), dan de hele module stoppen: de app toont dan
        # "fout" i.p.v. dat Vertel stil blijft hangen.
        RegisterEventHandler(OnProcessExit(target_action=llama, on_exit=[
            EmitEvent(event=Shutdown(reason='llama-server gestopt'))])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='auto', choices=['auto', *MODELS]),
        OpaqueFunction(function=server),
        Node(
            package='wave_rover_vision',
            executable='assistant',
            output='screen',
        ),
    ])
