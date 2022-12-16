from Problem import Problem
from dataclasses import dataclass, field
from typing import List, Optional

import gurobipy as gp


@dataclass
class TaskPlanner:
    name: str
    problem_definition: Problem
    model: gp.Model = field(init=False)

    def initialize(self):
        e = gp.Env(empty=True)
        if False:
            e.setParam('WLSACCESSID', wls_access_id)
            e.setParam('WLSSECRET', wls_secret)
            e.setParam('LICENSEID', license_id)
        e.start()

        # Create the model within the Gurobi environment
        self.model = gp.Model(self.name, env=e)

    def create_model(self):
        for task in self.problem_definition:
            for agent in task.get_agents():     #Note that if empty is a free param, it can be assigned to all existing agents
                #TODO: Add assignament constraints
            for precedence_tasks in task.get_precedence_constraints():
                #TODO: Add precedence constraints

    def add_problem(self, problem):
        # self.problem_definition.append(problem)
        self.problem_definition = problem
