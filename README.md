# RobotArm

MVP: Python GUI + IK + serial driver for 3 stepper joints (Arduino/TB6600).
Future: OpenRB gripper, CV, planning, simulation.

## Quickstart
```bash
python -m venv .venv
# Windows: .venv\Scripts\activate
pip install -U pip
pip install -e .
python scripts/run_gui.py

