#!/usr/bin/env python3
"""Vergelijkt beeldtaalmodellen voor de module scene op dezelfde foto's.

Start per model een tijdelijke llama-server (poort 8091, nooit twee tegelijk: zet de module
scene eerst uit) en stelt per foto dezelfde vragen als assistant (tool kijk): twee beschrijvingen,
een vrije vraag en een kader rond een voorwerp. Meet de tijd per antwoord en het geheugen.
In de container uitvoeren:

    docker compose exec ros python3 scripts/compare_vlm.py foto1.jpg[:voorwerp] ... \\
        [--models qwen3vl-2b qwen3vl-4b]

Na de dubbele punt het voorwerp waarvoor een kader gevraagd wordt (bv. foto.jpg:de vuilbak).
"""
import argparse
import base64
import json
import os
import signal
import subprocess
import time
import urllib.error
import urllib.request

from wave_rover_vision.assistant import DEFAULT_QUESTION, QWEN_SAMPLING
from wave_rover_vision.assistant import DESCRIBE_PROMPT as SYSTEM_PROMPT

VLM_DIR = '/home/ubuntu/wave_rover/models/vlm'
MODELS = {  # zoals in scene.launch.py
    'qwen3vl-2b': ('Qwen3VL-2B-Instruct-Q4_K_M.gguf', 'mmproj-Qwen3VL-2B-Instruct-Q8_0.gguf'),
    'qwen3vl-4b': ('Qwen3VL-4B-Instruct-Q4_K_M.gguf', 'mmproj-Qwen3VL-4B-Instruct-Q8_0.gguf'),
    'gemma3-4b': ('gemma-3-4b-it-Q4_K_M.gguf', 'mmproj-model-f16.gguf'),
}
URL = 'http://127.0.0.1:8091'


def available_mb():
    with open('/proc/meminfo') as f:
        for line in f:
            if line.startswith('MemAvailable'):
                return int(line.split()[1]) // 1024
    return -1


def ask(messages, max_tokens=100, **sampling):
    body = {'messages': messages, 'max_tokens': max_tokens, **(sampling or QWEN_SAMPLING)}
    req = urllib.request.Request(URL + '/v1/chat/completions', data=json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json'})
    start = time.monotonic()
    with urllib.request.urlopen(req, timeout=120) as r:
        text = json.load(r)['choices'][0]['message']['content']
    return ' '.join(text.split()), time.monotonic() - start


def test_model(name, photos):
    model, mmproj = MODELS[name]
    before = available_mb()
    server = subprocess.Popen(
        ['llama-server', '-m', f'{VLM_DIR}/{model}', '--mmproj', f'{VLM_DIR}/{mmproj}',
         '--n-gpu-layers', '99', '--ctx-size', '4096', '--parallel', '1', '--host', '127.0.0.1',
         '--port', '8091', '--no-webui'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        start = time.monotonic()
        while True:
            try:
                with urllib.request.urlopen(URL + '/health', timeout=1) as r:
                    if r.status == 200:
                        break
            except (urllib.error.URLError, OSError):
                pass
            if server.poll() is not None or time.monotonic() - start > 120:
                print(f'== {name}: start mislukt')
                return
            time.sleep(1)
        print(f'== {name}: geladen in {time.monotonic() - start:.0f} s, '
              f'{before - available_mb()} MB geheugen', flush=True)
        for path, thing in photos:
            with open(path, 'rb') as f:
                image = {'type': 'image_url', 'image_url': {
                    'url': 'data:image/jpeg;base64,' + base64.b64encode(f.read()).decode()}}
            print(f'-- {os.path.basename(path)}')
            for i in range(2):
                text, t = ask([{'role': 'system', 'content': SYSTEM_PROMPT},
                               {'role': 'user', 'content': [image, {'type': 'text', 'text': DEFAULT_QUESTION}]}])
                print(f'   beschrijving {i + 1} ({t:.1f} s): {text}', flush=True)
            text, t = ask([{'role': 'user', 'content': [image, {'type': 'text', 'text':
                           'Welke voorwerpen staan er op de vloer? Eén zin, in het Nederlands.'}]}])
            print(f'   vraag ({t:.1f} s): {text}', flush=True)
            if thing:
                text, t = ask([{'role': 'user', 'content': [image, {'type': 'text', 'text':
                               f'Zoek {thing} op de foto. Antwoord alleen met JSON: [{{"bbox_2d": '
                               f'[x1, y1, x2, y2], "label": "..."}}]. Is het er niet, antwoord dan [].'}]}],
                              max_tokens=80, temperature=0.0)
                print(f'   kader "{thing}" ({t:.1f} s): {text}', flush=True)
    finally:
        os.killpg(server.pid, signal.SIGTERM)
        server.wait(timeout=20)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('photos', nargs='+', help='foto.jpg of foto.jpg:voorwerp')
    parser.add_argument('--models', nargs='+', default=['qwen3vl-2b', 'qwen3vl-4b'])
    args = parser.parse_args()
    photos = [(p.split(':', 1) + [''])[:2] for p in args.photos]
    for name in args.models:
        test_model(name, photos)


if __name__ == '__main__':
    main()
