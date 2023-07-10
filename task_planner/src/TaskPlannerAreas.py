from Problem import Problem
from Task import TaskSolution

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from TaskPlanner import TaskPlanner

import gurobipy as gp
import itertools
import os
from utils import Objective

EPS = 1E-6
BIG_M = 1E7


@dataclass
class TaskPlannerAreas(TaskPlanner):

    def __post_init__(self):
        """
        The methos make sure that the TaskPlanner if is correctly defined: consistency check, requested solution >
        and if is able to solve respect that objective Returns:

        """
        if self.problem_definition.consistency_check() is not True:
            raise ValueError("The problem is not consistent. Check it!")
        if self.n_solutions < 1:
            raise ValueError("Unable to specify a number of solutions less than 1")
        if not isinstance(self.objective, Objective):
            raise ValueError("Objective should be an instance of Objective class")

    def create_model(self) -> None:
        """
        This method defines the decision variables and creates the basic model (t_end constraint and assignment constraint).

        Returns: None

        """
        try:
            neighboring_tasks = self.problem_definition.get_near_tasks()
        except ValueError():
            raise Exception()
        self.decision_variables["neighboring_tasks"] = self.model.addVars(neighboring_tasks,
                                                                          name="neighboring_tasks",
                                                                          vtype=gp.GRB.BINARY)
        print(self.decision_variables["neighboring_tasks"])
        super().create_model()
        self.add_neighboring_constraint()

    def add_neighboring_constraint(self) -> None:
        for (task_i, task_j) in self.decision_variables["neighboring_tasks"]:
            t_start_i = self.decision_variables["t_start"][task_i]
            t_start_j = self.decision_variables["t_start"][task_j]
            t_end_i = self.decision_variables["t_end"][task_i]
            t_end_j = self.decision_variables["t_end"][task_j]

            self.model.addConstr(t_start_j - t_end_i >= -BIG_M *
                                 (1 - self.decision_variables["neighboring_tasks"][(task_i, task_j)]))
            self.model.addConstr(t_start_i - t_end_j >= -BIG_M *
                                 (self.decision_variables["neighboring_tasks"][(task_i, task_j)]))
