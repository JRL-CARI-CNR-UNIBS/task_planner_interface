from basics.task import TaskSolution

from dataclasses import dataclass, field
from typing import List
from utils import Objective


from abc import ABC, abstractmethod


@dataclass
class TaskPlanner(ABC):
    objective: Objective = field(default=Objective.MAKESPAN)
    n_solutions: float = field(default=1)

    @abstractmethod
    def initialize(self) -> None:
        """
        This method initialize the TaskPlanner Object:
        Retrieve the licence if it exists,
        create the Model,
        define the solution number.

        """
        pass

    @abstractmethod
    def create_model(self) -> None:
        """
        This method defines the decision variables and creates the basic model (t_end constraint and assignment constraint).

        Returns: None

        """
        pass

    @abstractmethod
    def set_objective(self) -> None:
        pass

    @abstractmethod
    def solve(self) -> None:
        pass

    @abstractmethod
    def get_solution(self, solution_number: int = 0) -> List[TaskSolution]:
        pass
