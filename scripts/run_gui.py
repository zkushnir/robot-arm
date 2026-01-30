from pathlib import Path
import sys
import platform

# Support both PySide6 and PyQt5
try:
    from PySide6.QtWidgets import QApplication
except ImportError:
    from PyQt5.QtWidgets import QApplication

from armstack.core.config import load_robot_config
from armstack.ui.gui_qt import ArmGUI

def main():
    # Auto-detect platform and use appropriate config
    if len(sys.argv) > 1:
        cfg_path = Path(sys.argv[1])
    elif platform.system() == "Linux":
        cfg_path = Path("src/armstack/config/robot_description_jetson.yaml")
    else:
        cfg_path = Path("src/armstack/config/robot_description.yaml")

    cfg = load_robot_config(cfg_path)

    app = QApplication(sys.argv)
    w = ArmGUI(cfg)
    w.resize(1400, 800)
    w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
