#!/bin/bash
# Simple launcher for Jetson
cd "$(dirname "$0")"
export PYTHONPATH=$(pwd)/src
python3 scripts/run_gui.py
