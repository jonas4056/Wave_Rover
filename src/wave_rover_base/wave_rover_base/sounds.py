"""Geluidjes en deuntjes voor Wave, zelf gemaakt (geen opnames, geen rechten): glijdend fluitje,
boing, robotpiepjes, tadaa, en vrije kinderliedjes als robotdeuntje.

play() blokkeert tot het geluid gespeeld is (aplay op de USB-speaker, zoals speech.say) en geeft
een RuntimeError als de speaker er niet is of bezet is. Een geluid maken kost enkele ms numpy.
"""
import io
import random
import subprocess
import wave

import numpy as np

from wave_rover_base.speech import SPEAKER

RATE = 22050
VOLUME = 0.35  # van het maximum; de speaker staat zelf al op 40%

# Toonhoogtes (Hz) van de noten die de liedjes nodig hebben.
NOTES = {
    'G3': 196.0, 'A3': 220.0, 'B3': 246.9, 'C4': 261.6, 'D4': 293.7, 'E4': 329.6,
    'F4': 349.2, 'G4': 392.0, 'A4': 440.0, 'B4': 493.9, 'C5': 523.3, 'D5': 587.3,
    'E5': 659.3, 'F5': 698.5, 'G5': 784.0, 'C6': 1046.5,
}
# Vrije melodieën: traditioneel, en Happy Birthday (in Europa vrij sinds 2017); (noot, tellen),
# '-' = rust.
SONGS = {
    'Vader Jacob': [
        ('C4', 1), ('D4', 1), ('E4', 1), ('C4', 1), ('C4', 1), ('D4', 1), ('E4', 1), ('C4', 1),
        ('E4', 1), ('F4', 1), ('G4', 2), ('E4', 1), ('F4', 1), ('G4', 2),
        ('G4', 0.5), ('A4', 0.5), ('G4', 0.5), ('F4', 0.5), ('E4', 1), ('C4', 1),
        ('G4', 0.5), ('A4', 0.5), ('G4', 0.5), ('F4', 0.5), ('E4', 1), ('C4', 1),
        ('C4', 1), ('G3', 1), ('C4', 2), ('C4', 1), ('G3', 1), ('C4', 2)],
    'Altijd is Kortjakje ziek': [
        ('C4', 1), ('C4', 1), ('G4', 1), ('G4', 1), ('A4', 1), ('A4', 1), ('G4', 2),
        ('F4', 1), ('F4', 1), ('E4', 1), ('E4', 1), ('D4', 1), ('D4', 1), ('C4', 2),
        ('G4', 1), ('G4', 1), ('F4', 1), ('F4', 1), ('E4', 1), ('E4', 1), ('D4', 2),
        ('G4', 1), ('G4', 1), ('F4', 1), ('F4', 1), ('E4', 1), ('E4', 1), ('D4', 2),
        ('C4', 1), ('C4', 1), ('G4', 1), ('G4', 1), ('A4', 1), ('A4', 1), ('G4', 2),
        ('F4', 1), ('F4', 1), ('E4', 1), ('E4', 1), ('D4', 1), ('D4', 1), ('C4', 2)],
    'Happy Birthday': [
        ('G4', 0.75), ('G4', 0.25), ('A4', 1), ('G4', 1), ('C5', 1), ('B4', 2),
        ('G4', 0.75), ('G4', 0.25), ('A4', 1), ('G4', 1), ('D5', 1), ('C5', 2),
        ('G4', 0.75), ('G4', 0.25), ('G5', 1), ('E5', 1), ('C5', 1), ('B4', 1), ('A4', 2),
        ('F5', 0.75), ('F5', 0.25), ('E5', 1), ('C5', 1), ('D5', 1), ('C5', 2)],
}
# Kort deuntje om op te dansen (±4 s, net zo lang als het dansje).
DANCE_TUNE = [('C5', 0.5), ('E5', 0.5), ('G5', 0.5), ('E5', 0.5)] * 3 + [('C5', 1), ('C6', 1)]


def tone(f_start, f_end, seconds, shape='sine'):
    """Toon die van f_start naar f_end glijdt, met zachte aan- en uitzet (geen klikjes)."""
    n = int(RATE * seconds)
    freq = np.linspace(f_start, f_end, n)
    phase = 2 * np.pi * np.cumsum(freq) / RATE
    wave_ = np.sin(phase) if shape == 'sine' else np.sign(np.sin(phase)) * 0.4 + np.sin(phase) * 0.6
    fade = min(n // 2, int(RATE * 0.01))
    envelope = np.ones(n)
    if fade:
        envelope[:fade] = np.linspace(0, 1, fade)
        envelope[-fade:] = np.linspace(1, 0, fade)
    return wave_ * envelope


def silence(seconds):
    return np.zeros(int(RATE * seconds))


def melody(notes, beat=0.32, shape='square'):
    parts = []
    for note, beats in notes:
        length = beats * beat
        if note == '-':
            parts.append(silence(length))
        else:
            parts += [tone(NOTES[note], NOTES[note], length * 0.88, shape), silence(length * 0.12)]
    return np.concatenate(parts)


def effect(name):
    if name == 'wiew':  # glijdend fluitje: wiiieeeuw
        return np.concatenate([tone(500, 1600, 0.35), tone(1600, 400, 0.45)])
    if name == 'boing':
        t = np.arange(int(RATE * 0.5)) / RATE
        f = 180 + 120 * np.exp(-6 * t) * np.sin(2 * np.pi * 12 * t) + 300 * np.exp(-4 * t)
        return np.sin(2 * np.pi * np.cumsum(f) / RATE) * np.exp(-3 * t)
    if name == 'piep':  # robotgebabbel
        parts = []
        for _ in range(random.randint(4, 7)):
            a, b = random.uniform(900, 2800), random.uniform(900, 2800)
            parts += [tone(a, b if random.random() < 0.5 else a, random.uniform(0.05, 0.12)),
                      silence(random.uniform(0.02, 0.06))]
        return np.concatenate(parts)
    if name == 'tada':
        return melody([('C5', 0.5), ('E5', 0.5), ('G5', 0.5), ('C6', 2)], beat=0.18)
    raise KeyError(name)


def play(samples, speaker=SPEAKER):
    """Speelt een geluid (numpy, -1..1) af op de speaker."""
    pcm = (np.clip(samples, -1, 1) * VOLUME * 32767).astype('<i2').tobytes()
    data = io.BytesIO()
    with wave.open(data, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(pcm)
    result = subprocess.run(['aplay', '-q', '-D', speaker, '-t', 'wav', '-'],
                            input=data.getvalue(), capture_output=True, timeout=120)
    if result.returncode != 0:
        raise RuntimeError(f'speaker: {result.stderr.decode().strip() or result.returncode}')


def play_file(path, seconds=None, speaker=SPEAKER):
    """Speelt een WAV-bestand af (eigen muziek, scripts/add_music.sh), eventueel enkel het begin."""
    cmd = ['aplay', '-q', '-D', speaker] + (['-d', str(int(seconds))] if seconds else []) + [path]
    result = subprocess.run(cmd, capture_output=True, timeout=600)
    if result.returncode != 0:
        raise RuntimeError(f'speaker: {result.stderr.decode().strip() or result.returncode}')
