# from dataclasses import dataclass
from enum import Enum, auto

EPS = 1E-6
BIG_M = 1E7

class Behaviour(Enum):
    DISCRETE = 1
    CONTINUOUS = 2


class Overlapping(Enum):
    INSIDE = 1
    OUTSIDE = 2


class Objective(Enum):
    MAKESPAN = auto()
    SUM_T_START = auto()
    SUM_T_START_END = auto()
    SYNERGY = auto()
    SUM_T_END = auto()
    ACTUAL_MAKESPAN = auto()
    OTHER = auto()
