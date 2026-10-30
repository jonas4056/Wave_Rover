"""Tekst uitspreken via de USB-speaker: Piper (Vlaamse stem) -> aplay.

say() blokkeert tot de tekst uitgesproken is (Piper rekent ±1 s voor een zin van 6 s, aplay
speelt meteen af) en geeft een RuntimeError als de speaker er niet is. Gebruikt door
power_guard en assistant (wave_rover_vision). Piper staat in het image (/opt/piper),
de stemmen in ~/wave_rover/models/piper (scripts/download_scene_models.sh).
"""
import json
import subprocess

PIPER = '/opt/piper/piper'
VOICE = '/home/ubuntu/wave_rover/models/piper/nl_BE-nathalie-medium.onnx'
# ALSA-naam van de USB-speaker ("USB2.0 Device" -> kaart Device).
SPEAKER = 'plughw:CARD=Device,DEV=0'
# Afstelling van Piper (standaard van Piper): tempo (kleiner = sneller), variatie in toon en in
# klankduur (groter = levendiger), stilte na elke zin (s).
STYLE = {'length_scale': 1.0, 'noise_scale': 0.667, 'noise_w': 0.8, 'sentence_silence': 0.2}


def say(text, voice=VOICE, speaker=SPEAKER, piper=PIPER, style=None):
    """Piper maakt ruwe audio (16 bit mono), aplay speelt ze meteen af."""
    with open(voice + '.json') as f:
        rate = json.load(f)['audio']['sample_rate']
    options = [arg for name, value in {**STYLE, **(style or {})}.items()
               for arg in (f'--{name}', str(value))]
    synth = subprocess.Popen(
        [piper, '--model', voice, '--output-raw', *options],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    aplay = subprocess.Popen(
        ['aplay', '-q', '-D', speaker, '-r', str(rate), '-f', 'S16_LE', '-c', '1', '-t', 'raw'],
        stdin=synth.stdout, stderr=subprocess.PIPE)
    synth.stdout.close()  # aplay is nu de enige lezer
    synth.stdin.write(text.replace('\n', ' ').encode() + b'\n')
    synth.stdin.close()
    _, err = aplay.communicate(timeout=120)
    synth.wait(timeout=10)
    if aplay.returncode != 0:
        raise RuntimeError(f'speaker: {err.decode().strip() or aplay.returncode}')
