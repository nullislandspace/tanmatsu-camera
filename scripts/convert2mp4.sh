#!/bin/bash
# Convert AVI files recorded by the Tanmatsu camera app to MP4.
# The video stream (H.264) is copied without re-encoding.
# The audio stream (MP3) is copied without re-encoding.

set -euo pipefail

if [ $# -lt 1 ]; then
    echo "Usage: $0 <input.avi> [output.mp4]"
    exit 1
fi

INPUT="$1"

if [ ! -f "$INPUT" ]; then
    echo "Error: file not found: $INPUT"
    exit 1
fi

if [ $# -ge 2 ]; then
    OUTPUT="$2"
else
    OUTPUT="${INPUT%.avi}.mp4"
fi

ffmpeg -fflags +genpts -i "$INPUT" -c:v copy -c:a copy "$OUTPUT"
