from Problem import Problem
from dataclasses import dataclass, field
from typing import List, Dict, Optional

import gurobipy as gp
import itertools


@dataclass
class TaskPlanner:
    name: str
    problem_definition: Problem
    model: gp.Model = field(init=False)
    decision_variables: Dict[str, gp.tupledict] = field(init=False)

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
        agent_task_combination, cost = gp.multidict(self.problem_definition.get_combinations())
        print(agent_task_combination)
        print("--------------------------------------")
        #TODO: Ragionare sulle possibili combination
        # print(cost)
        tasks_list = self.problem_definition.get_tasks_list()
        agents = self.problem_definition.get_agents()
        # upper_bound = sum([max([cost[agent, task] for agent in agents]) for task in tasks_list]) * 1.5
        # self.decision_variables["assignment"] = self.model.addVars(agent_task_combination,
        #                                                            name="assignment",
        #                                                            vtype=gp.GRB.BINARY)
        # self.decision_variables["t_start"] = self.model.addVars(map(lambda task: f"{task}_start", tasks_list),
        #                                                         name="t_start",
        #                                                         vtype=gp.GRB.CONTINUOUS,
        #                                                         lb=0, ub=upper_bound)
        # self.decision_variables["t_end"] = self.model.addVars(map(lambda task: f"{task}_end", tasks_list),
        #                                                       name="t_end",
        #                                                       vtype=gp.GRB.CONTINUOUS,
        #                                                       lb=0, ub=upper_bound)
        group_of_two_x_agent = itertools.product(itertools.combinations(tasks_list, 2), agents)
        for k in group_of_two_x_agent:
            print(group_of_two_x_agent)
        # self.decision_variables["delta_ij"] = self.model.addVars(group_of_two_x_agent,
        #                                                          name="same_agent",
        #                                                          vtype=gp.GRB.BINARY)

        # for task in self.problem_definition:
        #     for agent in task.get_agents():     #Note that if empty is a free param, it can be assigned to all existing agents
        #         #TODO: Add assignament constraints
        #     for precedence_tasks in task.get_precedence_constraints():
        #         #TODO: Add precedence constraints
    # def add_constraints(self, problem):
    #     for task in self.problem_definition:
    #         for agent in task.get_agents():


    def add_problem(self, problem):
        # self.problem_definition.append(problem)
        self.problem_definition = problem
