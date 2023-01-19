from Problem import Problem
from Task import TaskSolution
from TaskPlanner import TaskPlanner

from dataclasses import dataclass, field
from typing import List, Dict, Optional

import gurobipy as gp
import itertools

EPS = 1E-6
BIG_M = 1E6


@dataclass
class TaskPlannerHumanAware(TaskPlanner):
    def create_model(self) -> None:
        tasks_list = self.problem_definition.get_tasks_list()
        tasks_pairs = itertools.combinations(tasks_list, 2)
        self.decision_variables["overlapping"] = self.model.addVars(tasks_pairs,
                                                                    name="overlapping",
                                                                    vtype=gp.GRB.CONTINUOUS,
                                                                    lb=-gp.GRB.INFINITY,
                                                                    ub=gp.GRB.INFINITY)
        super().create_model()
        print(self.decision_variables.keys())
        print(self.decision_variables.keys())
        self.add_overlapping_constraints()
        for task in tasks_list:
            self.model.addConstr(self.decision_variables["t_start"][task] >= 0)

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
        print("-----------------------------------------------")
        print(self.decision_variables.keys())
        print("-----------------------------------------------")
        # print(self.decision_variables["overlapping"])
        for task in self.decision_variables["t_start"].keys():
            for task_pair in self.decision_variables["overlapping"].keys():
                if task in task_pair:
                    parallel_task = set(task_pair).difference({task}).pop()
                    task_type = tasks_type_correspondence[task]
                    parallel_task_type = tasks_type_correspondence[parallel_task]
                    overlapping = self.decision_variables["overlapping"][task_pair]
                    if ("ur5_on_guide", task) in self.decision_variables["assignment"].keys():
                        assignment = self.decision_variables["assignment"][("ur5_on_guide", task)]

                        if (task_type, "ur5_on_guide", parallel_task, "human_right_arm") in tasks_synergies:
                            synergy = tasks_synergies[(task_type, "ur5_on_guide", parallel_task, "human_right_arm")]
                            t_end[task] += overlapping * (synergy - 1) * assignment
            self.model.addConstr(self.decision_variables["t_end"][task] == t_end[task], name=f"t_end({task})")
            # t_end = t_end[task] + gp.quicksum([self.decision_variables["overlapping"][task_pair]*(synergy_val())*self.decision_variables["assignment"][("ur5_on_guide",task)] for task_pair in self.decision_variables["overlapping"].keys() if task in task_pair])

    def get_solution(self) -> List[TaskSolution]:
        agents = self.problem_definition.get_agents()
        task_lists = self.problem_definition.get_tasks_list()

        problem_solution = []
        for task in task_lists:
            t_start = self.decision_variables["t_start"][task].X
            t_end = self.decision_variables["t_end"][task].X
            # assignments = self.decision_variables["assignment"].select("*", task)

            assignment = None
            for agent in agents:
                decision_variable = self.decision_variables["assignment"].select(agent, task)
                if decision_variable:
                    assert len(decision_variable) == 1
                    if len(decision_variable) == 1 and decision_variable[
                        0].X == 1:  # if len(decision_variable) == 1 and decision_variable[0].X == 1:
                        assignment = agent
            print(task, t_start, t_end, assignment)
            try:

                task_solution = self.problem_definition.add_task_solution(task, t_start, t_end, assignment)
                print(task_solution)
            except ValueError:
                print(f"Error during solution filling")
                raise Exception(f"T start of task: {task}, is negative: {t_start}, t_end: {t_end}, by: {assignment}")
            except Exception:
                print(f"Error during solution filling")
                raise Exception(f"{task}, not presence in problem task list")
            problem_solution.append(task_solution)
        return problem_solution
