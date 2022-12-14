from Task import Task
from dataclasses import dataclass
from typing import List, Optional

@dataclass
class TaskPlanner:
    name: str
    task_list: List[str]

    def initialize(self):
        pass

    def