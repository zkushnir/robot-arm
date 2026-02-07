import math

# Support both PySide6 and PyQt5
try:
    from PySide6.QtCore import Qt, QPointF, QRectF, Signal
    from PySide6.QtGui import QPainter, QPen, QColor, QBrush, QPainterPath
    from PySide6.QtWidgets import (
        QWidget, QVBoxLayout, QHBoxLayout, QLabel,
        QSlider, QDoubleSpinBox, QPushButton, QTextEdit, QGroupBox, QSpinBox, QDial, QComboBox
    )
except ImportError:
    from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal as Signal
    from PyQt5.QtGui import QPainter, QPen, QColor, QBrush, QPainterPath
    from PyQt5.QtWidgets import (
        QWidget, QVBoxLayout, QHBoxLayout, QLabel,
        QSlider, QDoubleSpinBox, QPushButton, QTextEdit, QGroupBox, QSpinBox, QDial, QComboBox
    )

from armstack.core.config import RobotConfig
from armstack.kinematics.planar_2link import ik_2link_planar
from armstack.drivers.arduino_stepper import ArduinoStepperDriver

def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi

def deg2rad(d: float) -> float:
    return d * math.pi / 180.0


class ArmVisualization2D(QWidget):
    """2D visualization of the robot arm"""

    def __init__(self, L1: float, L2: float, view_type: str = "side"):
        super().__init__()
        self.L1 = L1  # mm
        self.L2 = L2  # mm
        self.view_type = view_type  # "side" or "top"

        # Current angles (degrees)
        self.base_angle = 0.0
        self.shoulder_angle = 0.0
        self.elbow_angle = 0.0

        # Target position
        self.target_x = None
        self.target_y = None

        # Make it scalable for fullscreen
        self.setMinimumSize(200, 200)
        try:
            from PySide6.QtWidgets import QSizePolicy
        except ImportError:
            from PyQt5.QtWidgets import QSizePolicy
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def update_angles(self, base: float, shoulder: float, elbow: float):
        """Update arm angles (degrees)"""
        self.base_angle = base
        self.shoulder_angle = shoulder
        self.elbow_angle = elbow
        self.update()

    def update_target(self, x: float, y: float):
        """Update target position (mm)"""
        self.target_x = x
        self.target_y = y
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Background
        painter.fillRect(self.rect(), QColor(20, 20, 20))

        # Get widget dimensions
        w = self.width()
        h = self.height()
        center_x = w / 2
        center_y = h / 2

        # Scale factor (pixels per mm)
        max_reach = self.L1 + self.L2
        scale = min(w, h) * 0.35 / max_reach

        if self.view_type == "side":
            self._draw_side_view(painter, center_x, center_y, scale)
        else:
            self._draw_top_view(painter, center_x, center_y, scale)

    def _draw_side_view(self, painter: QPainter, cx: float, cy: float, scale: float):
        """Draw side view of arm (shoulder and elbow joints)"""

        # Draw workspace circle
        max_reach = (self.L1 + self.L2) * scale
        painter.setPen(QPen(QColor(40, 40, 40), 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(cx, cy), max_reach, max_reach)

        # Draw grid
        painter.setPen(QPen(QColor(30, 30, 30), 1))
        for i in range(-int(max_reach), int(max_reach), 50):
            painter.drawLine(int(cx + i), 0, int(cx + i), int(cy * 2))
            painter.drawLine(0, int(cy + i), int(cx * 2), int(cy + i))

        # Calculate joint positions
        # Base at (cx, cy)
        # Add 90 degrees to make 0° point straight up instead of horizontal
        theta1_rad = deg2rad(self.shoulder_angle + 90)
        theta2_rad = deg2rad(self.elbow_angle)

        # Shoulder position (base)
        base_x = cx
        base_y = cy

        # Elbow position
        elbow_x = base_x + self.L1 * math.cos(theta1_rad) * scale
        elbow_y = base_y - self.L1 * math.sin(theta1_rad) * scale

        # End effector position
        end_x = elbow_x + self.L2 * math.cos(theta1_rad + theta2_rad) * scale
        end_y = elbow_y - self.L2 * math.sin(theta1_rad + theta2_rad) * scale

        # Draw target if set
        if self.target_x is not None and self.target_y is not None:
            target_px = cx + self.target_x * scale
            target_py = cy - self.target_y * scale
            painter.setPen(QPen(QColor(160, 32, 240, 100), 2, Qt.DashLine))
            painter.setBrush(QBrush(QColor(160, 32, 240, 50)))
            painter.drawEllipse(QPointF(target_px, target_py), 15, 15)
            painter.setPen(QPen(QColor(160, 32, 240), 1))
            painter.drawText(int(target_px + 20), int(target_py), "Target")

        # Draw links
        # Link 1 (base to elbow)
        painter.setPen(QPen(QColor(160, 32, 240), 8, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(QPointF(base_x, base_y), QPointF(elbow_x, elbow_y))

        # Link 2 (elbow to end effector)
        painter.setPen(QPen(QColor(200, 100, 255), 8, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(QPointF(elbow_x, elbow_y), QPointF(end_x, end_y))

        # Draw joints
        # Base joint
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.setBrush(QBrush(QColor(80, 80, 80)))
        painter.drawEllipse(QPointF(base_x, base_y), 10, 10)

        # Elbow joint
        painter.setBrush(QBrush(QColor(100, 100, 100)))
        painter.drawEllipse(QPointF(elbow_x, elbow_y), 8, 8)

        # End effector
        painter.setBrush(QBrush(QColor(160, 32, 240)))
        painter.drawEllipse(QPointF(end_x, end_y), 10, 10)

        # Draw labels
        painter.setPen(QPen(QColor(160, 32, 240), 1))
        painter.drawText(10, 20, f"Side View")
        painter.drawText(10, 40, f"Shoulder: {self.shoulder_angle:.1f}°")
        painter.drawText(10, 60, f"Elbow: {self.elbow_angle:.1f}°")

    def _draw_top_view(self, painter: QPainter, cx: float, cy: float, scale: float):
        """Draw top view of arm (base rotation)"""

        # Draw workspace circle
        max_reach = (self.L1 + self.L2) * scale
        painter.setPen(QPen(QColor(40, 40, 40), 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(cx, cy), max_reach, max_reach)

        # Draw grid circles
        painter.setPen(QPen(QColor(30, 30, 30), 1))
        for r in range(50, int(max_reach), 50):
            painter.drawEllipse(QPointF(cx, cy), r, r)

        # Draw angle lines
        for angle in range(0, 360, 30):
            rad = deg2rad(angle)
            x = cx + max_reach * math.cos(rad)
            y = cy + max_reach * math.sin(rad)
            painter.drawLine(QPointF(cx, cy), QPointF(x, y))

        # Calculate arm projection from top
        # Add 90 degrees to make 0° point up (forward) instead of right
        base_rad = deg2rad(self.base_angle + 90)
        # Add 90 degrees to make 0° point straight up
        theta1_rad = deg2rad(self.shoulder_angle + 90)
        theta2_rad = deg2rad(self.elbow_angle)

        # Project onto XY plane
        # Arm reach in XY plane
        arm_reach_xy = self.L1 * math.cos(theta1_rad) + self.L2 * math.cos(theta1_rad + theta2_rad)

        # End effector position in top view
        end_x = cx + arm_reach_xy * math.cos(base_rad) * scale
        end_y = cy - arm_reach_xy * math.sin(base_rad) * scale

        # Draw base rotation indicator
        painter.setPen(QPen(QColor(160, 32, 240), 3))
        painter.drawLine(QPointF(cx, cy), QPointF(end_x, end_y))

        # Draw base
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.setBrush(QBrush(QColor(80, 80, 80)))
        painter.drawEllipse(QPointF(cx, cy), 15, 15)

        # Draw end effector
        painter.setBrush(QBrush(QColor(160, 32, 240)))
        painter.drawEllipse(QPointF(end_x, end_y), 10, 10)

        # Draw angle arc (starts from top/up position)
        painter.setPen(QPen(QColor(200, 100, 255), 2))
        painter.setBrush(Qt.NoBrush)
        rect = QRectF(cx - 40, cy - 40, 80, 80)
        # Start at 90° (top/up) and draw in the correct direction
        painter.drawArc(rect, 90 * 16, int(self.base_angle * 16))

        # Draw labels
        painter.setPen(QPen(QColor(160, 32, 240), 1))
        painter.drawText(10, 20, f"Top View")
        painter.drawText(10, 40, f"Base: {self.base_angle:.1f}°")

class ArmGUI(QWidget):
    def __init__(self, cfg: RobotConfig):
        super().__init__()
        self.setWindowTitle("KushBot - Robot Arm Control")

        raw = cfg.raw
        self.L1 = float(raw["links"]["L1_mm"])
        self.L2 = float(raw["links"]["L2_mm"])
        self.port = raw["serial"]["arduino_port"]
        self.baud = int(raw["serial"]["arduino_baud"])

        self.driver = ArduinoStepperDriver(self.port, self.baud)
        self.connected = False

        # Current end effector position (calculated from FK)
        self.current_ee_x = 0.0
        self.current_ee_y = 0.0
        self.current_ee_z = 0.0

        # Keyboard control settings
        self.keyboard_step_size = 10.0  # mm

        # Apply dark theme
        self._apply_dark_theme()

        # Create visualizations
        self.side_view = ArmVisualization2D(self.L1, self.L2, "side")
        self.top_view = ArmVisualization2D(self.L1, self.L2, "top")

        # Create main layout
        main_layout = QVBoxLayout()

        # Header with KushBot branding
        header = self._create_header()
        main_layout.addWidget(header)

        # Connection section
        conn_group = self._create_connection_section()
        main_layout.addWidget(conn_group)

        # Main content: controls on left, visualizations on right
        content_layout = QHBoxLayout()

        # Left side: controls
        left_layout = QVBoxLayout()

        # End Effector Position Display
        ee_pos_group = self._create_ee_position_section()
        left_layout.addWidget(ee_pos_group)

        # Direct angle control section
        angle_group = self._create_angle_control_section()
        left_layout.addWidget(angle_group)

        # IK control section
        ik_group = self._create_ik_control_section()
        left_layout.addWidget(ik_group)

        # Action buttons section
        action_group = self._create_action_section()
        left_layout.addWidget(action_group)

        # Keyboard controls help
        keyboard_help = self._create_keyboard_help()
        left_layout.addWidget(keyboard_help)

        content_layout.addLayout(left_layout, 1)

        # Right side: visualizations
        viz_layout = QVBoxLayout()
        viz_layout.addWidget(self.side_view)
        viz_layout.addWidget(self.top_view)
        content_layout.addLayout(viz_layout, 1)

        main_layout.addLayout(content_layout)

        # Log section
        log_group = QGroupBox("📝 Log Output")
        log_layout = QVBoxLayout()
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(120)
        log_layout.addWidget(self.log)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

        self.setLayout(main_layout)
        self.write("🚀 KushBot ready. Click 'Connect Arduino' when plugged in.")
        self.write("⌨️ Use arrow keys + W/S for keyboard control after connecting")

    def _apply_dark_theme(self):
        """Apply dark theme with purple accents"""
        self.setStyleSheet("""
            QWidget {
                background-color: #0a0a0a;
                color: #a020f0;
                font-family: 'Segoe UI', Arial, sans-serif;
                font-size: 11pt;
            }
            QGroupBox {
                border: 2px solid #a020f0;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 15px;
                font-weight: bold;
                color: #a020f0;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 5px 10px;
                color: #a020f0;
            }
            QLabel {
                color: #d896ff;
            }
            QPushButton {
                background-color: #1a1a1a;
                border: 2px solid #a020f0;
                border-radius: 6px;
                padding: 8px 16px;
                color: #a020f0;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #a020f0;
                color: #0a0a0a;
            }
            QPushButton:pressed {
                background-color: #8010d0;
            }
            QPushButton:disabled {
                background-color: #1a1a1a;
                border-color: #404040;
                color: #404040;
            }
            QTextEdit {
                background-color: #0f0f0f;
                border: 2px solid #a020f0;
                border-radius: 6px;
                color: #d896ff;
                padding: 8px;
            }
            QSpinBox, QDoubleSpinBox {
                background-color: #1a1a1a;
                border: 2px solid #a020f0;
                border-radius: 4px;
                padding: 4px;
                color: #d896ff;
            }
            QSpinBox::up-button, QDoubleSpinBox::up-button {
                border-left: 1px solid #a020f0;
                background-color: #2a2a2a;
            }
            QSpinBox::down-button, QDoubleSpinBox::down-button {
                border-left: 1px solid #a020f0;
                background-color: #2a2a2a;
            }
            QDial {
                background-color: #1a1a1a;
            }
            QComboBox {
                background-color: #1a1a1a;
                border: 2px solid #a020f0;
                border-radius: 4px;
                padding: 4px;
                color: #d896ff;
            }
            QComboBox:hover {
                border-color: #d896ff;
            }
            QComboBox::drop-down {
                border-left: 1px solid #a020f0;
                background-color: #2a2a2a;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 4px solid transparent;
                border-right: 4px solid transparent;
                border-top: 4px solid #a020f0;
            }
            QComboBox QAbstractItemView {
                background-color: #1a1a1a;
                border: 2px solid #a020f0;
                selection-background-color: #a020f0;
                selection-color: #0a0a0a;
                color: #d896ff;
            }
        """)

    def _create_header(self):
        """Create KushBot branding header"""
        header = QWidget()
        layout = QVBoxLayout()

        # Main title
        title = QLabel("⚡ KUSHBOT ⚡")
        title.setStyleSheet("""
            font-size: 32pt;
            font-weight: bold;
            color: #a020f0;
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 #a020f0, stop:0.5 #d896ff, stop:1 #a020f0);
            -webkit-background-clip: text;
            padding: 10px;
        """)
        title.setAlignment(Qt.AlignCenter)

        # Subtitle
        subtitle = QLabel("3-DOF Robot Arm Control System")
        subtitle.setStyleSheet("font-size: 12pt; color: #d896ff;")
        subtitle.setAlignment(Qt.AlignCenter)

        layout.addWidget(title)
        layout.addWidget(subtitle)
        header.setLayout(layout)
        return header

    def _create_ee_position_section(self):
        """Create end effector position display and control"""
        group = QGroupBox("📍 End Effector Position (Real-time)")
        layout = QVBoxLayout()

        # Position display (read-only, updated automatically)
        pos_layout = QHBoxLayout()

        self.ee_x_display = QLabel("X: 0.0 mm")
        self.ee_y_display = QLabel("Y: 0.0 mm")
        self.ee_z_display = QLabel("Z: 0.0 mm")

        for label in [self.ee_x_display, self.ee_y_display, self.ee_z_display]:
            label.setStyleSheet("""
                font-size: 14pt;
                font-weight: bold;
                color: #00ff00;
                background-color: #0f0f0f;
                border: 2px solid #a020f0;
                border-radius: 4px;
                padding: 8px;
            """)
            pos_layout.addWidget(label)

        layout.addLayout(pos_layout)

        # Override controls (for manual input)
        override_layout = QHBoxLayout()
        override_label = QLabel("Manual Override:")
        override_label.setStyleSheet("font-size: 10pt; color: #d896ff;")

        self.ee_x_override = QDoubleSpinBox()
        self.ee_y_override = QDoubleSpinBox()
        self.ee_z_override = QDoubleSpinBox()

        for spinbox in [self.ee_x_override, self.ee_y_override, self.ee_z_override]:
            spinbox.setRange(-500, 500)
            spinbox.setSingleStep(1.0)
            spinbox.setDecimals(1)
            spinbox.setMinimumWidth(80)

        override_btn = QPushButton("Move to Override Position")
        override_btn.clicked.connect(self._move_to_override_position)

        override_layout.addWidget(override_label)
        override_layout.addWidget(QLabel("X:"))
        override_layout.addWidget(self.ee_x_override)
        override_layout.addWidget(QLabel("Y:"))
        override_layout.addWidget(self.ee_y_override)
        override_layout.addWidget(QLabel("Z:"))
        override_layout.addWidget(self.ee_z_override)
        override_layout.addWidget(override_btn)

        layout.addLayout(override_layout)

        group.setLayout(layout)
        return group

    def _create_keyboard_help(self):
        """Create keyboard controls legend"""
        group = QGroupBox("⌨️ Keyboard Controls")
        layout = QVBoxLayout()

        help_text = QLabel(
            "🔼 Up Arrow: Forward (+Y)\n"
            "🔽 Down Arrow: Backward (-Y)\n"
            "◀️ Left Arrow: Left (-X)\n"
            "▶️ Right Arrow: Right (+X)\n"
            "🔤 W: Up (+Z)\n"
            "🔤 S: Down (-Z)\n"
            "Step size: 10mm per key press"
        )
        help_text.setStyleSheet("font-size: 10pt; color: #d896ff; line-height: 1.6;")

        layout.addWidget(help_text)
        group.setLayout(layout)
        return group

    def _create_connection_section(self):
        group = QGroupBox("🔌 Connection")
        layout = QHBoxLayout()

        self.status_label = QLabel(f"⚪ Disconnected")
        self.status_label.setStyleSheet("font-size: 12pt; font-weight: bold;")

        # Port selection dropdown
        port_label = QLabel("Port:")
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        self._populate_ports()
        self.port_combo.currentTextChanged.connect(self._on_port_changed)

        # Refresh ports button
        refresh_btn = QPushButton("🔄")
        refresh_btn.setMaximumWidth(40)
        refresh_btn.clicked.connect(self._populate_ports)
        refresh_btn.setToolTip("Refresh port list")

        self.connect_btn = QPushButton("🔗 Connect Arduino")
        self.connect_btn.clicked.connect(self.connect_arduino)

        layout.addWidget(self.status_label)
        layout.addWidget(port_label)
        layout.addWidget(self.port_combo)
        layout.addWidget(refresh_btn)
        layout.addWidget(self.connect_btn)
        layout.addStretch()

        group.setLayout(layout)
        return group

    def _populate_ports(self):
        """Populate the port dropdown with available serial ports"""
        import serial.tools.list_ports

        current_port = self.port_combo.currentText() if hasattr(self, 'port_combo') else self.port
        self.port_combo.clear()

        # Get available ports
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in sorted(ports)]

        if not port_list:
            port_list = [self.port]  # Add configured port as fallback

        self.port_combo.addItems(port_list)

        # Try to select the previously selected port
        if current_port in port_list:
            self.port_combo.setCurrentText(current_port)
        elif self.port in port_list:
            self.port_combo.setCurrentText(self.port)

        # Only log if log widget exists (not during initialization)
        if hasattr(self, 'log'):
            self.write(f"📡 Found {len(port_list)} serial port(s)")

    def _on_port_changed(self, new_port):
        """Handle port selection change"""
        if new_port:
            self.port = new_port
            # Only log if log widget exists (not during initialization)
            if hasattr(self, 'log'):
                self.write(f"🔧 Port changed to: {new_port}")

    def _create_angle_control_section(self):
        group = QGroupBox("🎯 Direct Motor Angle Control")
        layout = QVBoxLayout()

        # Create a horizontal layout for the three dials
        dials_layout = QHBoxLayout()

        # Base motor dial
        base_container = QVBoxLayout()
        base_container.addWidget(QLabel("Base"), alignment=Qt.AlignCenter)
        self.base_dial = QDial()
        self.base_dial.setRange(-180, 180)
        self.base_dial.setValue(0)
        self.base_dial.setNotchesVisible(True)
        self.base_dial.setWrapping(False)
        self.base_dial.valueChanged.connect(self._update_visualization)
        base_container.addWidget(self.base_dial)
        self.base_angle_label = QLabel("0°")
        self.base_angle_label.setAlignment(Qt.AlignCenter)
        self.base_angle_label.setStyleSheet("font-size: 14pt; font-weight: bold;")
        base_container.addWidget(self.base_angle_label)
        dials_layout.addLayout(base_container)

        # Shoulder motor dial
        shoulder_container = QVBoxLayout()
        shoulder_container.addWidget(QLabel("Shoulder"), alignment=Qt.AlignCenter)
        self.shoulder_dial = QDial()
        self.shoulder_dial.setRange(-180, 180)
        self.shoulder_dial.setValue(0)
        self.shoulder_dial.setNotchesVisible(True)
        self.shoulder_dial.setWrapping(False)
        self.shoulder_dial.valueChanged.connect(self._update_visualization)
        shoulder_container.addWidget(self.shoulder_dial)
        self.shoulder_angle_label = QLabel("0°")
        self.shoulder_angle_label.setAlignment(Qt.AlignCenter)
        self.shoulder_angle_label.setStyleSheet("font-size: 14pt; font-weight: bold;")
        shoulder_container.addWidget(self.shoulder_angle_label)
        dials_layout.addLayout(shoulder_container)

        # Elbow motor dial
        elbow_container = QVBoxLayout()
        elbow_container.addWidget(QLabel("Elbow"), alignment=Qt.AlignCenter)
        self.elbow_dial = QDial()
        self.elbow_dial.setRange(-180, 180)
        self.elbow_dial.setValue(0)
        self.elbow_dial.setNotchesVisible(True)
        self.elbow_dial.setWrapping(False)
        self.elbow_dial.valueChanged.connect(self._update_visualization)
        elbow_container.addWidget(self.elbow_dial)
        self.elbow_angle_label = QLabel("0°")
        self.elbow_angle_label.setAlignment(Qt.AlignCenter)
        self.elbow_angle_label.setStyleSheet("font-size: 14pt; font-weight: bold;")
        elbow_container.addWidget(self.elbow_angle_label)
        dials_layout.addLayout(elbow_container)

        layout.addLayout(dials_layout)

        # Send button
        self.send_angles_btn = QPushButton("▶ Move to Angles")
        self.send_angles_btn.clicked.connect(self.send_angles)
        layout.addWidget(self.send_angles_btn)

        group.setLayout(layout)
        return group

    def _update_visualization(self):
        """Update visualization when dials change"""
        # Invert all dial values (turning left increases, turning right decreases)
        base = -self.base_dial.value()
        shoulder = -self.shoulder_dial.value()
        elbow = -self.elbow_dial.value()

        # Update labels with inverted values
        self.base_angle_label.setText(f"{base}°")
        self.shoulder_angle_label.setText(f"{shoulder}°")
        self.elbow_angle_label.setText(f"{elbow}°")

        # Calculate and update end effector position
        self._calculate_forward_kinematics(base, shoulder, elbow)
        self._update_ee_display()

        # Update visualizations
        self.side_view.update_angles(base, shoulder, elbow)
        self.top_view.update_angles(base, shoulder, elbow)

    def _create_ik_control_section(self):
        group = QGroupBox("🎯 Target Position Control (IK)")
        layout = QVBoxLayout()

        # X position
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel("X Position:"))
        self.x_in = QDoubleSpinBox()
        self.x_in.setRange(-500, 500)
        self.x_in.setValue(150)
        self.x_in.setSuffix(" mm")
        self.x_in.valueChanged.connect(self._update_ik_target)
        x_layout.addWidget(self.x_in)
        layout.addLayout(x_layout)

        # Y position
        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel("Y Position:"))
        self.y_in = QDoubleSpinBox()
        self.y_in.setRange(-500, 500)
        self.y_in.setValue(50)
        self.y_in.setSuffix(" mm")
        self.y_in.valueChanged.connect(self._update_ik_target)
        y_layout.addWidget(self.y_in)
        layout.addLayout(y_layout)

        # Base rotation
        base_layout = QHBoxLayout()
        base_layout.addWidget(QLabel("Base Rotation:"))
        self.base_ik = QSpinBox()
        self.base_ik.setRange(-180, 180)
        self.base_ik.setValue(0)
        self.base_ik.setSuffix(" deg")
        base_layout.addWidget(self.base_ik)
        layout.addLayout(base_layout)

        # Send button
        self.send_ik_btn = QPushButton("🎯 Move to Position (IK)")
        self.send_ik_btn.clicked.connect(self.send_ik)
        layout.addWidget(self.send_ik_btn)

        group.setLayout(layout)
        return group

    def _update_ik_target(self):
        """Update visualization with target position"""
        x = self.x_in.value()
        y = self.y_in.value()
        self.side_view.update_target(x, y)

    def _create_action_section(self):
        group = QGroupBox("⚡ Actions")
        layout = QHBoxLayout()

        # Speed control
        layout.addWidget(QLabel("Speed:"))
        self.speed_in = QDoubleSpinBox()
        self.speed_in.setRange(1, 360)
        self.speed_in.setValue(250)
        self.speed_in.setSuffix(" deg/s")
        layout.addWidget(self.speed_in)

        layout.addStretch()

        # Home button
        self.home_btn = QPushButton("🏠 HOME")
        self.home_btn.clicked.connect(self.home)
        layout.addWidget(self.home_btn)

        # Zero button
        self.zero_btn = QPushButton("⚙️ ZERO")
        self.zero_btn.clicked.connect(self.zero)
        layout.addWidget(self.zero_btn)

        # Stop button
        self.stop_btn = QPushButton("⛔ STOP")
        self.stop_btn.clicked.connect(self.stop)
        layout.addWidget(self.stop_btn)

        group.setLayout(layout)
        return group

    def write(self, s: str):
        self.log.append(s)

    def _calculate_forward_kinematics(self, base_deg: float, shoulder_deg: float, elbow_deg: float):
        """Calculate end effector position from joint angles"""
        # Convert to radians
        base_rad = deg2rad(base_deg)
        shoulder_rad = deg2rad(shoulder_deg + 90)  # Add 90 since 0 is vertical
        elbow_rad = deg2rad(elbow_deg)

        # Calculate 2D arm position (in plane of shoulder/elbow)
        elbow_x = self.L1 * math.cos(shoulder_rad)
        elbow_y = self.L1 * math.sin(shoulder_rad)

        end_x_2d = elbow_x + self.L2 * math.cos(shoulder_rad + elbow_rad)
        end_y_2d = elbow_y + self.L2 * math.sin(shoulder_rad + elbow_rad)

        # Project to 3D with base rotation
        self.current_ee_x = end_x_2d * math.cos(base_rad)
        self.current_ee_y = end_x_2d * math.sin(base_rad)
        self.current_ee_z = end_y_2d

    def _update_ee_display(self):
        """Update end effector position display labels"""
        self.ee_x_display.setText(f"X: {self.current_ee_x:.1f} mm")
        self.ee_y_display.setText(f"Y: {self.current_ee_y:.1f} mm")
        self.ee_z_display.setText(f"Z: {self.current_ee_z:.1f} mm")

        # Also update override spinboxes to show current position
        self.ee_x_override.setValue(self.current_ee_x)
        self.ee_y_override.setValue(self.current_ee_y)
        self.ee_z_override.setValue(self.current_ee_z)

    def _move_to_override_position(self):
        """Move to manually entered override position"""
        x = self.ee_x_override.value()
        y = self.ee_y_override.value()
        z = self.ee_z_override.value()

        self.write(f"🎯 Moving to override position: X={x:.1f}, Y={y:.1f}, Z={z:.1f}")

        # Calculate distance in XY plane
        r = math.sqrt(x*x + y*y)

        # Calculate base angle
        base_deg = rad2deg(math.atan2(y, x))

        # Use IK for 2D arm in vertical plane
        result = ik_2link_planar(r, z, self.L1, self.L2, elbow_up=True)

        if not result.ok:
            self.write(f"❌ IK failed: {result.message}")
            return

        shoulder_deg = rad2deg(result.theta1_rad) - 90  # Subtract 90 since 0 is vertical
        elbow_deg = rad2deg(result.theta2_rad)

        self.write(f"📐 IK solution: base={base_deg:.1f}°, shoulder={shoulder_deg:.1f}°, elbow={elbow_deg:.1f}°")

        # Update dials (with inversion)
        self.base_dial.setValue(-int(base_deg))
        self.shoulder_dial.setValue(-int(shoulder_deg))
        self.elbow_dial.setValue(-int(elbow_deg))

        # Send to robot
        self.send_angles()

    def keyPressEvent(self, event):
        """Handle keyboard input for IMMEDIATE end effector control"""
        if not self.connected:
            self.write("⚠️ Connect Arduino first to use keyboard control")
            return

        # Determine movement direction
        dx, dy, dz = 0, 0, 0

        key = event.key()
        if key == Qt.Key_Up:
            dy = self.keyboard_step_size  # Forward (+Y)
        elif key == Qt.Key_Down:
            dy = -self.keyboard_step_size  # Backward (-Y)
        elif key == Qt.Key_Left:
            dx = -self.keyboard_step_size  # Left (-X)
        elif key == Qt.Key_Right:
            dx = self.keyboard_step_size  # Right (+X)
        elif key == Qt.Key_W or key == Qt.Key_W - 32:  # W or w
            dz = self.keyboard_step_size  # Up (+Z)
        elif key == Qt.Key_S or key == Qt.Key_S - 32:  # S or s
            dz = -self.keyboard_step_size  # Down (-Z)
        else:
            return  # Ignore other keys

        # Calculate new target position from current position
        new_x = self.current_ee_x + dx
        new_y = self.current_ee_y + dy
        new_z = self.current_ee_z + dz

        # Log the command
        direction = ""
        if dy > 0:
            direction = "⬆️ Forward"
        elif dy < 0:
            direction = "⬇️ Backward"
        elif dx > 0:
            direction = "➡️ Right"
        elif dx < 0:
            direction = "⬅️ Left"
        elif dz > 0:
            direction = "🔼 Up"
        elif dz < 0:
            direction = "🔽 Down"

        self.write(f"{direction} → Target: ({new_x:.1f}, {new_y:.1f}, {new_z:.1f})")

        # Execute movement immediately via IK
        self._execute_keyboard_move(new_x, new_y, new_z)

    def _execute_keyboard_move(self, x: float, y: float, z: float):
        """Execute immediate movement to target position"""
        # Calculate distance in XY plane
        r = math.sqrt(x*x + y*y)

        # Calculate base angle
        base_deg = rad2deg(math.atan2(y, x))

        # Use IK for 2D arm in vertical plane
        result = ik_2link_planar(r, z, self.L1, self.L2, elbow_up=True)

        if not result.ok:
            self.write(f"❌ Target unreachable: {result.message}")
            return

        shoulder_deg = rad2deg(result.theta1_rad) - 90  # Subtract 90 since 0 is vertical
        elbow_deg = rad2deg(result.theta2_rad)

        # Send command immediately to robot
        speed = float(self.speed_in.value())

        # Negate base and elbow for correct motor direction
        self.driver.move_joints_deg(-base_deg, shoulder_deg, -elbow_deg, speed)

        # Update visualization (dials) to match
        self.base_dial.setValue(-int(base_deg))
        self.shoulder_dial.setValue(-int(shoulder_deg))
        self.elbow_dial.setValue(-int(elbow_deg))

    def connect_arduino(self):
        if self.connected:
            self.write("⚠️ Already connected.")
            return
        try:
            # Reinitialize driver with selected port
            self.driver = ArduinoStepperDriver(self.port, self.baud)
            self.driver.connect()
            self.connected = True
            self.write(f"✅ Connected: {self.port} @ {self.baud}")
            self.status_label.setText(f"🟢 Connected - {self.port}")
            self.connect_btn.setText("✅ Connected")
            self.connect_btn.setEnabled(False)
            self.port_combo.setEnabled(False)
            for ln in self.driver.read_lines(10):
                self.write(f"🤖 ARD: {ln}")
        except Exception as e:
            self.write(f"❌ Connect failed: {e}")
            self.status_label.setText(f"🔴 Connection Failed - {self.port}")

    def send_angles(self):
        if not self.connected:
            self.write("⚠️ Not connected. Please connect to Arduino first.")
            return

        # Invert all dial values
        base = -float(self.base_dial.value())
        shoulder = -float(self.shoulder_dial.value())
        elbow = -float(self.elbow_dial.value())
        speed = float(self.speed_in.value())

        self.write(f"▶️ Moving to angles: base={base:.1f}°, shoulder={shoulder:.1f}°, elbow={elbow:.1f}° @ {speed:.1f} deg/s")

        try:
            # Negate base and elbow for correct motor direction
            self.driver.move_joints_deg(-base, shoulder, -elbow, speed)
            for ln in self.driver.read_lines(10):
                self.write(f"🤖 ARD: {ln}")
        except Exception as e:
            self.write(f"❌ Error sending angles: {e}")

    def send_ik(self):
        x = float(self.x_in.value())
        y = float(self.y_in.value())
        base_deg = float(self.base_ik.value())
        speed = float(self.speed_in.value())

        res = ik_2link_planar(x, y, self.L1, self.L2, elbow_up=True)
        if not res.ok:
            self.write(f"❌ IK FAIL: {res.message}")
            return

        shoulder_deg = rad2deg(res.theta1_rad)
        elbow_deg = rad2deg(res.theta2_rad)
        self.write(f"🎯 IK Solution → base={base_deg:.2f}°, shoulder={shoulder_deg:.2f}°, elbow={elbow_deg:.2f}°")

        # Update dials to show calculated angles (inverted for dial direction)
        self.base_dial.setValue(-int(base_deg))
        self.shoulder_dial.setValue(-int(shoulder_deg))
        self.elbow_dial.setValue(-int(elbow_deg))

        if not self.connected:
            self.write("⚠️ Not connected. Please connect to Arduino first.")
            return

        try:
            # Negate base and elbow for correct motor direction
            self.driver.move_joints_deg(-base_deg, shoulder_deg, -elbow_deg, speed)
            for ln in self.driver.read_lines(10):
                self.write(f"🤖 ARD: {ln}")
        except Exception as e:
            self.write(f"❌ Error sending IK command: {e}")

    def home(self):
        if not self.connected:
            self.write("⚠️ Not connected. Please connect to Arduino first.")
            return

        speed = float(self.speed_in.value())
        self.write(f"🏠 Homing all motors (moving to 0 degrees) @ {speed:.1f} deg/s")

        try:
            self.driver.home_all(speed)
            # Reset dials to 0
            self.base_dial.setValue(0)
            self.shoulder_dial.setValue(0)
            self.elbow_dial.setValue(0)
            for ln in self.driver.read_lines(10):
                self.write(f"🤖 ARD: {ln}")
        except Exception as e:
            self.write(f"❌ Error during home: {e}")

    def zero(self):
        if not self.connected:
            self.write("⚠️ Not connected. Please connect to Arduino first.")
            return

        self.write("⚙️ Zeroing all motors (setting current position as 0 degrees)")

        try:
            self.driver.zero_all()
            # Reset dials to 0
            self.base_dial.setValue(0)
            self.shoulder_dial.setValue(0)
            self.elbow_dial.setValue(0)
            for ln in self.driver.read_lines(10):
                self.write(f"🤖 ARD: {ln}")
        except Exception as e:
            self.write(f"❌ Error during zero: {e}")

    def stop(self):
        if self.connected:
            self.driver.stop()
            self.write("⛔ STOP sent - motors disabled")
        else:
            self.write("⚠️ Not connected.")

    def closeEvent(self, event):
        try:
            self.driver.close()
        except Exception:
            pass
        event.accept()
