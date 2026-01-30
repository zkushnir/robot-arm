import time
from typing import Optional
import serial

class ArduinoStepperDriver:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.1):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

    def connect(self):
        self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=self.timeout)
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        if self.ser:
            self.ser.close()
        self.ser = None

    def send_line(self, line: str):
        if not self.ser:
            raise RuntimeError("Arduino not connected")
        self.ser.write((line.strip() + "\n").encode("utf-8"))

    def read_lines(self, max_lines: int = 20):
        if not self.ser:
            return []
        out = []
        for _ in range(max_lines):
            raw = self.ser.readline()
            if not raw:
                break
            out.append(raw.decode("utf-8", errors="replace").strip())
        return out

    def move_joints_deg(self, base, shoulder, elbow, speed_deg_s=60.0):
        self.send_line(f"MOVE {base:.3f} {shoulder:.3f} {elbow:.3f} {speed_deg_s:.3f}")

    def stop(self):
        self.send_line("STOP")

    def home_all(self, speed_deg_s: float = 60.0):
        """Move all motors to 0 degrees (home position)"""
        self.send_line(f"HOMEALL {speed_deg_s:.3f}")

    def zero_all(self):
        """Reset all motor positions to 0 (set current position as 0)"""
        self.send_line("ZEROALL")

    def get_state(self):
        """Request current state from Arduino"""
        self.send_line("GET")
