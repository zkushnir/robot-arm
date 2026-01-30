#!/bin/bash
# Setup script for Jetson with Python 3.6

echo "Installing system packages..."
sudo apt-get update
sudo apt-get install -y python3-numpy python3-pyqt5

echo "Installing Python packages..."
pip3 install --user pyserial pyyaml

echo "Setup complete!"
echo ""
echo "To run the GUI:"
echo "  PYTHONPATH=$(pwd)/src python3 scripts/run_gui.py"
echo ""
echo "Or create an alias by adding this to ~/.bashrc:"
echo "  alias robot-gui='cd $(pwd) && PYTHONPATH=$(pwd)/src python3 scripts/run_gui.py'"
