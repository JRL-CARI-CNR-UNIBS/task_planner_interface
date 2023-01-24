from Problem import Problem
from Task import TaskSolution
from TaskPlannerHumanAware import TaskPlannerHumanAware

from dataclasses import dataclass, field
from typing import List, Dict, Optional
from utils import Behaviour, Objective

import gurobipy as gp
import itertools

EPS = 1E-12
BIG_M = 1E12


@dataclass
class TaskPlannerHumanAwareEasier(TaskPlannerHumanAware):
    def add_t_end_constraints(self, agent_task_combination, cost) -> None:
        super(TaskPlannerHumanAware, self).add_t_end_constraints(agent_task_combination, cost)

    def set_objective(self) -> None:
        cost_function = self.model.addVar(name="J", vtype=gp.GRB.CONTINUOUS)


        sigma_index = self.model.addVar(name="sigma_index", vtype=gp.GRB.CONTINUOUS)

        parallel_agent = "human_right_arm"
        main_agent = "ur5_on_guide"
        # TODO: Set two agents in "init"
        tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
        tasks_synergies = self.problem_definition.get_tasks_synergies()

        sigma_val = 0.0
        sigma_tot = 0.0
        _, cost = gp.multidict(self.problem_definition.get_combinations())
        for task in self.decision_variables["t_start"].keys():
            for task_pair in self.decision_variables["overlapping"].keys():
                if task in task_pair:
                    parallel_task = set(task_pair).difference({task}).pop()
                    task_type = tasks_type_correspondence[task]
                    parallel_task_type = tasks_type_correspondence[parallel_task]
                    overlapping = self.decision_variables["overlapping"][task_pair]
                    if (main_agent, task) in self.decision_variables["assignment"].keys() and \
                            (parallel_agent, parallel_task) in self.decision_variables["assignment"].keys():
                        assignment_main = self.decision_variables["assignment"][(main_agent, task)]
                        # assignment_parallel = self.decision_variables["assignment"][(parallel_agent, parallel_task)]
                        if (task_type, main_agent, parallel_task_type, parallel_agent) in tasks_synergies:
                            synergy = tasks_synergies[(task_type, main_agent, parallel_task_type, parallel_agent)]
                            # if (main_agent, parallel_task) in cost.keys(): Se discreto ci vuole il cost
                            if self.behaviour == Behaviour.CONTINUOUS:
                                sigma_val += overlapping * (synergy - 1) * assignment_main
                            elif self.behaviour == Behaviour.DISCRETE:
                                sigma_val += overlapping * (synergy - 1) * assignment_main * cost[(main_agent, task)]
                            sigma_tot += synergy
        self.model.addConstr(sigma_index == sigma_val / 1)

        cost_function = self.model.addVar(name="cost_function", vtype=gp.GRB.CONTINUOUS)
        cost_function_temp = self.model.addVar(name="cost_function_temp", vtype=gp.GRB.CONTINUOUS)

        if self.objective == Objective.SUM_T_START_END:
            self.model.addConstr(cost_function_temp == gp.quicksum(self.decision_variables["t_end"]) +
                                 gp.quicksum(self.decision_variables["t_start"]))
            self.model.addConstr(cost_function == cost_function_temp + sigma_index )
        elif self.objective == Objective.MAKESPAN:
            self.model.addConstr(cost_function_temp == gp.max_(self.decision_variables["t_end"]) )
            self.model.addConstr(cost_function == cost_function_temp + sigma_index)
        elif self.objective == Objective.SUM_T_START:
            self.model.addConstr(cost_function_temp == gp.quicksum(self.decision_variables["t_start"]))
            self.model.addConstr(cost_function == cost_function_temp + sigma_index)
        elif self.objective == Objective.SUM_T_END:
            self.model.addConstr(cost_function_temp == gp.quicksum(self.decision_variables["t_end"]))
            self.model.addConstr(cost_function == cost_function_temp + sigma_index)
        elif self.objective == Objective.SYNERGY:
            self.model.addConstr(cost_function_temp == gp.quicksum(self.decision_variables["t_end"]))
            self.model.addConstr(cost_function == sigma_index + cost_function_temp * 0.5)

        self.model.setObjective(cost_function, gp.GRB.MINIMIZE)
