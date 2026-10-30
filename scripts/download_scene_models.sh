#!/bin/bash
# Downloadt de modellen voor de module scene naar ~/wave_rover/models (niet in git):
# - vlm/: beeldtaalmodellen (GGUF) voor llama-server: Qwen3-VL-2B (standaard), Qwen3-VL-4B en Gemma 3 4B
# - piper/: Nederlandse stemmen voor Piper (Vlaams: Nathalie, rdh; Nederlands: Alex, Pim, Ronnie, mls met 52 sprekers)
# Op de Jetson uitvoeren: bash ~/wave_rover/scripts/download_scene_models.sh
set -e

MODELS="$HOME/wave_rover/models"
HF=https://huggingface.co

get() {  # get <map> <url>: alleen downloaden als het bestand er nog niet staat
    local file="$MODELS/$1/$(basename "$2")"
    mkdir -p "$MODELS/$1"
    [ -f "$file" ] && return
    echo "-> $1/$(basename "$2")"
    # Eerst als .part: een afgebroken download hervat de volgende keer (-C -).
    curl -fL --retry 3 -C - -o "$file.part" "$2"
    mv "$file.part" "$file"
}

get vlm "$HF/Qwen/Qwen3-VL-2B-Instruct-GGUF/resolve/main/Qwen3VL-2B-Instruct-Q4_K_M.gguf"
get vlm "$HF/Qwen/Qwen3-VL-2B-Instruct-GGUF/resolve/main/mmproj-Qwen3VL-2B-Instruct-Q8_0.gguf"
# Qwen3-VL-4B: zelfde familie (kan ook kaders geven), beter Nederlands, ±2x trager (nog te testen)
get vlm "$HF/Qwen/Qwen3-VL-4B-Instruct-GGUF/resolve/main/Qwen3VL-4B-Instruct-Q4_K_M.gguf"
get vlm "$HF/Qwen/Qwen3-VL-4B-Instruct-GGUF/resolve/main/mmproj-Qwen3VL-4B-Instruct-Q8_0.gguf"
get vlm "$HF/ggml-org/gemma-3-4b-it-GGUF/resolve/main/gemma-3-4b-it-Q4_K_M.gguf"
get vlm "$HF/ggml-org/gemma-3-4b-it-GGUF/resolve/main/mmproj-model-f16.gguf"

for voice in nl_BE/nathalie nl_BE/rdh nl_NL/alex nl_NL/pim nl_NL/ronnie nl_NL/mls; do
    for ext in onnx onnx.json; do
        get piper "$HF/rhasspy/piper-voices/resolve/main/nl/$voice/medium/${voice%%/*}-${voice##*/}-medium.$ext"
    done
done
ls -lh "$MODELS/vlm" "$MODELS/piper"
