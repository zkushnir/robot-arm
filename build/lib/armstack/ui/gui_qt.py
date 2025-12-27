from __future__ import annotations
import math

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QDoubleSpinBox, QPushButton, QTextEdit
)

from armstack.core.config import RobotConfig
from armstack.kinematics.planar_2link import ik_2link_planar
from armstack.drivers.arduino_stepper import ArduinoStepperDriver

def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi

class ArmGUI(QWidget):
    def __init__(self, cfg: RobotConfig):
        super().__init__()
        self.setWindowTitle("Robot Arm Control (MVP)")

        raw = cfg.raw
        self.L1 = float(raw["links"]["L1_mm"])
        self.L2 = float(raw["links"]["L2_mm"])
        self.port = raw["serial"]["arduino_port"]
        self.baud = int(raw["serial"]["arduino_baud"])

        self.driver = ArduinoStepperDriver(self.port, self.baud)
        self.connected = False

        self.log = QTextEdit()
        self.log.setReadOnly(True)

        self.x_in = QDoubleSpinBox(); self.x_in.setRange(-500, 500); self.x_in.setValue(150)
        self.y_in = QDoubleSpinBox(); self.y_in.setRange(-500, 500); self.y_in.setValue(50)
        self.base = QSlider(Qt.Horizontal); self.base.setRange(-180, 180); self.base.setValue(0)
        self.speed_in = QDoubleSpinBox(); self.speed_in.setRange(1, 360); self.speed_in.setValue(60)

        self.connect_btn = QPushButton("Connect Arduino")
        self.connect_btn.clicked.connect(self.connect_arduino)

        self.send_btn = QPushButton("Send IK Target")
        self.send_btn.clicked.connect(self.send_ik)

        self.stop_btn = QPushButton("STOP")
        self.stop_btn.clicked.connect(self.stop)

        row = QHBoxLayout()
        row.addWidget(QLabel("X (mm)")); row.addWidget(self.x_in)
        row.addWidget(QLabel("Y (mm)")); row.addWidget(self.y_in)
        row.addWidget(QLabel("Base (deg)")); row.addWidget(self.base)
        row.addWidget(QLabel("Speed")); row.addWidget(self.speed_in)
        row.addWidget(self.connect_btn)
        row.addWidget(self.send_btn)
        row.addWidget(self.stop_btn)

        layout = QVBoxLayout()
        layout.addLayout(row)
        layout.addWidget(self.log)
        self.setLayout(layout)

        self.write("GUI ready. Click 'Connect Arduino' when plugged in.")

    def write(self, s: str):
        self.log.append(s)

    def connect_arduino(self):
        if self.connected:
            self.write("Already connected.")
            return
        try:
            self.driver.connect()
            self.connected = True
            self.write(f"Connected: {self.port} @ {self.baud}")
            for ln in self.driver.read_lines(10):
                self.write(f"ARD: {ln}")
        except Exception as e:
            self.write(f"Connect failed: {e}")

    def send_ik(self):
        x = float(self.x_in.value())
        y = float(self.y_in.value())
        base_deg = float(self.base.value())
        speed = float(self.speed_in.value())

        res = ik_2link_planar(x, y, self.L1, self.L2, elbow_up=True)
        if not res.ok:
            self.write(f"IK FAIL: {res.message}")
            return

        shoulder_deg = rad2deg(res.theta1_rad)
        elbow_deg = rad2deg(res.theta2_rad)
        self.write(f"IK -> base={base_deg:.2f}, sh={shoulder_deg:.2f}, el={elbow_deg:.2f}")

        if not self.connected:
            self.write("Not connected: showing angles only.")
            return

        self.driver.move_joints_deg(base_deg, shoulder_deg, elbow_deg, speed)
        for ln in self.driver.read_lines(10):
            self.write(f"ARD: {ln}")

    def stop(self):
        if self.connected:
            self.driver.stop()
            self.write("STOP sent")
        else:
            self.write("Not connected.")

    def closeEvent(self, event):
        try:
            self.driver.close()
        except Exception:
            pass
        event.accept()
