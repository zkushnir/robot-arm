# RobotArm 🤖

A modular, research-grade robotic arm platform built from the ground up to support
**kinematics, motion planning, computer vision, and learned control**.

This project is designed as both a **physical robotic system** and a **software stack**
that scales from direct joint control to higher-level autonomy.  
The long-term goal is a tight integration between **hardware, perception, and intelligence**.
<p align="center">
  <img src="docs/images/robot_arm_cad.png" width="600">
</p>

---

## ✨ Key Features

- **3-DOF stepper-driven arm**
  - TB6600 drivers
  - Cycloidal gear reduction
  - Smooth acceleration-controlled motion

- **Multi-DOF gripper**
  - 6× DYNAMIXEL motors (pinching fingers + wrist orientation)
  - Controlled via OpenRB-150

- **Python control stack**
  - Inverse kinematics (IK)
  - Qt GUI for interactive control
  - Clean serial protocol to embedded controllers

- **Designed for scalability**
  - Future modules for:
    - Computer vision (camera-based grasping)
    - Motion planning & trajectory generation
    - Simulation / digital twin
    - Learned policies & AI-driven behaviors

---

## 🧠 System Architecture

High-level control is handled in Python, while low-level real-time motor control is
delegated to embedded controllers.

---

## 🖥️ Software Stack

- **Python**
  - IK, control logic, GUI, future CV & planning
- **Qt (PySide6)**
  - Desktop GUI
- **Arduino**
  - Stepper pulse generation + safety
- **DYNAMIXEL SDK**
  - Smart actuator control

---

## 🚀 Quick Start

### Prerequisites
- Python 3.10 or higher
- Arduino Uno (or compatible) with stepper motor drivers
- USB cable for Arduino connection

### Installation

**Option 1: Install as editable package (recommended for development)**
```bash
# Create virtual environment
python -m venv .venv

# Activate virtual environment
# Windows:
.venv\Scripts\activate
# macOS/Linux:
# source .venv/bin/activate

# Install dependencies
pip install -U pip
pip install -e .
```

**Option 2: Install from requirements.txt**
```bash
pip install -r requirements.txt
```

### Upload Arduino Firmware

1. Open Arduino IDE
2. Open `firmware/arduino_stepper_controller/arduino_stepper_controller.ino`
3. Select your board (Tools → Board → Arduino Uno)
4. Select your COM port (Tools → Port)
5. Upload the firmware (Ctrl+U)
6. Close Arduino IDE (important - only one program can access the serial port)

### Launch GUI

**Windows:**
```bash
# Easy way - double-click
run_gui.bat

# Or run directly
python scripts/run_gui.py
```

**macOS/Linux:**
```bash
python scripts/run_gui.py
```

### Configuration

Edit `src/armstack/config/robot_description.yaml` to match your setup:
```yaml
links:
  L1_mm: 120.0  # Shoulder to elbow link length
  L2_mm: 120.0  # Elbow to end effector link length

serial:
  arduino_port: "COM7"  # Change to your Arduino port
  arduino_baud: 115200
```

---

## 🎮 Using the GUI

The GUI provides intuitive control with real-time visualization:

### Features
- **Dark theme with purple accents** for easy on the eyes
- **Circular dial controls** for natural motor angle adjustment
- **2D visualizations** showing side and top views of the arm
- **Inverse kinematics** for end effector position control
- **Real-time feedback** with Arduino response logging

### Controls
1. **Connect** - Click "Connect Arduino" to establish serial connection
2. **Direct Control** - Use the circular dials to set joint angles
3. **IK Control** - Enter X,Y coordinates for automatic angle calculation
4. **HOME** - Return all motors to 0° position
5. **ZERO** - Calibrate current position as 0°
6. **STOP** - Emergency stop (disables all motors)

