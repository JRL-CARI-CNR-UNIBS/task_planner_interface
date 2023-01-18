from Problem import Problem
from Task import TaskSolution
from TaskPlanner import TaskPlanner

from dataclasses import dataclass, field
from typing import List, Dict, Optional

import gurobipy as gp
import itertools

EPS = 1E-6
BIG_M = 1E7


@dataclass
class TaskPlannerHumanAware(TaskPlanner):
    def create_model(self) -> None:
        super().create_model()
        tasks_list = self.problem_definition.get_tasks_list()
        tasks_pairs = itertools.combinations(tasks_list, 2)
        self.decision_variables["overlapping"] = self.model.addVars(tasks_pairs,
                                                                    name="overlapping",
                                                                    vtype=gp.GRB.CONTINUOUS,
                                                                    lb=-gp.GRB.INFINITY,
                                                                    ub=gp.GRB.INFINITY)
        self.add_overlapping_constraints()

    def add_overlapping_constraints(self):
        for task_i, task_j in self.decision_variables["overlapping"].keys():
            t_end_i = self.decision_variables["t_end"][task_i]
            t_end_j = self.decision_variables["t_end"][task_j]
            t_start_i = self.decision_variables["t_start"][task_i]
            t_start_j = self.decision_variables["t_start"][task_j]

            check_parallelism = self.model.addVar(name=f"raw_overlapping({task_i},{task_j})",
                                                  vtype=gp.GRB.CONTINUOUS,
                                                  lb=-gp.GRB.INFINITY,
                                                  ub=gp.GRB.INFINITY)
            min_t_end = self.model.addVar(name=f"min_t_end({task_i}, {task_j})",
                                          vtype=gp.GRB.CONTINUOUS)
            max_t_start = self.model.addVar(name=f"max_t_start({task_i}, {task_j})",
                                            vtype=gp.GRB.CONTINUOUS)
            self.model.addConstr(min_t_end == gp.min_(t_end_i, t_end_j))
            self.model.addConstr(max_t_start == gp.max_(t_start_i, t_start_j))
            self.model.addConstr(check_parallelism == (min_t_end - max_t_start))

            check_parallelism_geq_zero = self.model.addVar(name="check_parallelism_geq_zero_" + task_i + "_" + task_j,
                                                           vtype=gp.GRB.BINARY, lb=0, ub=1)

            self.model.addConstr(check_parallelism >= -BIG_M * (1 - check_parallelism_geq_zero),
                                 name="aux_1" + task_i + "_" + task_j)
            self.model.addConstr(check_parallelism <= BIG_M * check_parallelism_geq_zero - EPS,
                                 name="aux_2" + task_i + "_" + task_j)

            self.model.addConstr((self.decision_variables["overlapping"][(task_i, task_j)] == (
                        min_t_end - max_t_start) * check_parallelism_geq_zero))

    def add_t_end_constraints(self, agent_task_combination, cost) -> None:
        t_end = {}
        # print(agent_task_combination)
        for agent, task in agent_task_combination:
            if task not in t_end:
                t_end[task] = self.decision_variables["t_start"][task]
            t_end[task] += self.decision_variables["assignment"][(agent, task)] * cost[(agent, task)]
            # t_end[task] +=
        tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
        tasks_synergies = self.problem_definition.get_tasks_synergies()

        parallel_agent = "human_right_arm"
        main_agent = "ur5_on_guide"
        for task in self.decision_variables["t_start"].keys():
            for task_pair in self.decision_variables["overlapping"].keys():
                if task in task_pair:
                    parallel_task = set(task_pair).difference({task}).pop()
                    task_type = tasks_type_correspondence[task]
                    parallel_task_type = tasks_type_correspondence[parallel_task]
                    overlapping = self.decision_variables["overlapping"][task_pair]
                    assignment = self.decision_variables["assignment"][("ur5_on_guide",task)]

                    if (task_type, "ur5_on_guide", parallel_task, "human_right_arm") in tasks_synergies:
                        synergy = tasks_synergies[(task_type, "ur5_on_guide", parallel_task, "human_right_arm")]
                        #TODO: Continuare da qui.
                    self.dec/ision_variables["overlapping"][task_pair]*
                    # sinergia[task,]
                    #la sinergia del task con l'altro di task pair
        #
        #
            t_end = t_end[task] + gp.quicksum([self.decision_variables["overlapping"][task_pair]*(synergy_val())*self.decision_variables["assignment"][("ur5_on_guide",task)] for task_pair in self.decision_variables["overlapping"].keys() if task in task_pair])
        #     print(task)
        # for a,b in self.decision_variables["overlapping"].keys():
        #     print(a)
        #     print(b)
        # Tend constraints
        # t_end = {}
        # for agent, task in agent_task_combination:
        #     if task not in t_end:
        #         t_end[task] = self.decision_variables["t_start"][task]
        #     t_end[task] += self.decision_variables["assignment"][(agent, task)] * cost[(agent, task)]
        #
        # for task in self.problem_definition.:
        #
        #
        # for task in tasks_list:
        #     self.model.addConstr(t_end[task + "_end"] == t_end_incremental[task] + quicksum(
        #         [overlapping[keys] * (synergy_val[(task, set(keys).difference({task}).pop())] - 1) *assignment self.decision_variables["assignment"][("Robot", task)]
        #          for keys in overlapping.keys() if task in keys]))

    # def add_constraints(self) -> None:
    #     super().add_constraints()
    # tasks_per_agent = self.problem_definition.get_tasks_per_agent()
    # couple_of_two_tasks = itertools.combinations(tasks_per_agent[agent], 2)
    #
    # check_parallelism = self.model.addVar(name="check_parallelism_between_{}_{}".format(task_i, task_j), vtype=GRB.CONTINUOUS,
    #                              lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)
    #
    # min_t_end = self.model.addVar(name="min_t_end" + task_i + task_j, vtype=gp.GRB.CONTINUOUS)
    # max_t_start = self.model.addVar(name="max_t_start" + task_i + task_j, vtype=gp.GRB.CONTINUOUS)
    # self.model.addConstr(min_t_end == gp.min_(t_end_i, t_end_j))
    # self.model.addConstr(check_parallelism == (min_t_end - max_t_start))
    # self.model.addConstr(max_t_start == gp.max_(t_start_i, t_start_j))
    #
    # check_parallelism_geq_zero = self.model.addVar(name="check_parallelism_geq_zero_" + task_i + "_" + task_j, vtype=GRB.BINARY,
    #                                       lb=0, ub=1)
    #
    # self.model.addConstr(check_parallelism >= -BIG_M * (1 - check_parallelism_geq_zero), name="aux_1" + task_i + "_" + task_j)
    # self.model.addConstr(check_parallelism <= BIG_M * check_parallelism_geq_zero - EPS, name="aux_2" + task_i + "_" + task_j)
    #
    # self.model.addConstr((self.decision_variables["overlapping"][(task_i, task_j)] == (min_t_end - max_t_start) * check_parallelism_geq_zero))
