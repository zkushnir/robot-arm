# RealSense Vision Setup

Guide for setting up Intel RealSense camera for block detection and manipulation.

## Windows Setup (Development)

### 1. Install RealSense SDK

Download and install the Intel RealSense SDK:
- Go to: https://github.com/IntelRealSense/librealsense/releases
- Download the latest Windows installer
- Run the installer

### 2. Install Python Dependencies

```bash
pip install pyrealsense2 opencv-python
```

Or install all requirements:
```bash
pip install -r requirements.txt
```

### 3. Test Camera

Plug in your RealSense camera and run:

```bash
python scripts/test_realsense.py
```

**Controls:**
- `q` - Quit
- `r/g/b/y` - Switch color detection (red/green/blue/yellow)
- `d` - Toggle debug visualization
- `s` - Save current frame

## Jetson Setup (Deployment)

### 1. Install RealSense SDK on Jetson

```bash
# Install dependencies
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev

# Build librealsense from source (required for Jetson)
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3
make -j4
sudo make install

# Install Python bindings
pip3 install pyrealsense2
```

### 2. Install OpenCV

```bash
sudo apt-get install python3-opencv
```

### 3. Test on Jetson

```bash
PYTHONPATH=/home/kush/projects/robot-arm/src python3 scripts/test_realsense.py
```

## Calibrating Color Detection

The default color ranges may not work perfectly for your block. To tune them:

1. Run the test script with debug mode (`d` key)
2. Look at the mask in the top-left corner
3. If detection is poor, adjust the HSV ranges in `block_detector.py`

### HSV Color Tuning

Colors are defined in HSV (Hue, Saturation, Value):
- **Hue**: 0-180 (color)
- **Saturation**: 0-255 (color intensity)
- **Value**: 0-255 (brightness)

Example ranges:
- Red: H=0-10 or 160-180, S=100-255, V=100-255
- Blue: H=100-130, S=100-255, V=100-255
- Green: H=40-80, S=100-255, V=100-255
- Yellow: H=20-40, S=100-255, V=100-255

## 3D Coordinate System

The RealSense camera returns 3D points in **camera frame**:
- **X**: Right (meters)
- **Y**: Down (meters)
- **Z**: Forward / depth (meters)

You'll need to transform these to robot frame for motion planning.

## Next Steps

1. Test camera and block detection on Windows
2. Tune color ranges for your specific block
3. Implement camera-to-robot coordinate transformation
4. Integrate with robot control for pick-and-place
5. Deploy to Jetson
