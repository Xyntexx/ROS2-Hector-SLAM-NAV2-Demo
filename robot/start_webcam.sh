#!/bin/bash
# Start webcam stream server at 5 fps
cd "$(dirname "$0")"
exec python3 webcam_stream.py "$@"
