from pathlib import Path
import sys

# Support both PySide6 and PyQt5
try:
    from PySide6.QtWidgets import QApplication
except ImportError:
    from PyQt5.QtWidgets import QApplication

from armstack.core.config import load_robot_config
from armstack.ui.gui_qt import ArmGUI

def main():
    cfg_path = Path("src/armstack/config/robot_description.yaml")
    cfg = load_robot_config(cfg_path)

    app = QApplication(sys.argv)
    w = ArmGUI(cfg)
    w.resize(1400, 800)
    w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
