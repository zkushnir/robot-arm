from dataclasses import dataclass
from pathlib import Path
from typing import Union
import yaml

@dataclass
class RobotConfig:
    raw: dict

def load_robot_config(path: Union[str, Path]) -> RobotConfig:
    p = Path(path)
    data = yaml.safe_load(p.read_text(encoding="utf-8"))
    return RobotConfig(raw=data)
