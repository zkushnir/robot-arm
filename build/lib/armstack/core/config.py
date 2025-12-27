from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import yaml

@dataclass
class RobotConfig:
    raw: dict

def load_robot_config(path: str | Path) -> RobotConfig:
    p = Path(path)
    data = yaml.safe_load(p.read_text(encoding="utf-8"))
    return RobotConfig(raw=data)
