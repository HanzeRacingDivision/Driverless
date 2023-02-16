from typing import Dict


class Robot:
    RobotState: Dict
    WorldState: Dict

    def __init__(self):
        self.RobotState = {}
        self.WorldState = {}