#!/bin/bash
# Zet een eigen muziekbestand (mp3, m4a, wav, ...) op de robot zodat Wave het kan afspelen
# ("speel <naam>", of "dans op <naam>"). Vanaf de pc uitvoeren:
#
#   bash scripts/add_music.sh ~/Muziek/dai-dai.mp3 "dai dai"
#
# De naam (tweede argument, anders de bestandsnaam) is wat je tegen Wave zegt. Omzetten
# gebeurt met GStreamer op de Jetson (mono, 22050 Hz WAV, zoals de stem) naar
# ~/wave_rover/models/muziek/<naam>.wav (niet in git). Alleen voor eigen gebruik thuis.
set -e

SRC="$1"
if [ ! -f "$SRC" ]; then
    echo "Gebruik: bash scripts/add_music.sh <bestand> [naam]" >&2
    exit 1
fi
NAME="${2:-$(basename "${SRC%.*}")}"
# Bestandsnaam: accenten weg (qué -> que), kleine letters, spaties -> streepjes.
NAME="$(echo "$NAME" | iconv -f utf-8 -t ascii//TRANSLIT | tr '[:upper:]' '[:lower:]' \
        | tr ' _' '--' | tr -cd 'a-z0-9-')"

scp -q "$SRC" "jetson:/tmp/wave_muziek_in"
ssh jetson "mkdir -p ~/wave_rover/models/muziek && gst-launch-1.0 -q filesrc location=/tmp/wave_muziek_in \
    ! decodebin ! audioconvert ! audioresample \
    ! audio/x-raw,rate=22050,channels=1,format=S16LE ! wavenc \
    ! filesink location=\$HOME/wave_rover/models/muziek/$NAME.wav && rm -f /tmp/wave_muziek_in \
    && ls -lh ~/wave_rover/models/muziek/$NAME.wav"
echo "Klaar: zeg tegen Wave \"speel ${NAME//-/ }\" of \"dans op ${NAME//-/ }\""
