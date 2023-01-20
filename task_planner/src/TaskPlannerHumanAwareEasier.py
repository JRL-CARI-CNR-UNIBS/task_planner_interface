from Problem import Problem
from Task import TaskSolution
from TaskPlanner import TaskPlanner

from dataclasses import dataclass, field
from typing import List, Dict, Optional

import gurobipy as gp
import itertools

EPS = 1E-12
BIG_M = 1E12


@dataclass
class TaskPlannerHumanAwareEasier(TaskPlanner):
    def create_model(self) -> None:
        tasks_list = self.problem_definition.get_tasks_list()
        tasks_pairs = itertools.combinations(tasks_list, 2)
        upper_bound = self.problem_definition.get_nominal_upper_bound() * 10

        self.decision_variables["overlapping"] = self.model.addVars(tasks_pairs,
                                                                    name="overlapping",
                                                                    vtype=gp.GRB.BINARY,
                                                                    lb=0,
                                                                    ub=1)
        super().create_model()
        self.add_overlapping_constraints()
        # self.model.setParam("MIPGapAbs", 0.28)


    def set_objective(self) -> None:
        makespan = self.model.addVar(name="makespan", vtype=gp.GRB.CONTINUOUS)
        self.model.addConstr(makespan == gp.max_(self.decision_variables["t_end"]))

        sigma_index = self.model.addVar(name="sigma_index", vtype=gp.GRB.CONTINUOUS)
        parallel_agent = "human_right_arm"
        main_agent = "ur5_on_guide"
        tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
        tasks_synergies = self.problem_definition.get_tasks_synergies()

        sigma_val = 0
        sigma_tot = 0
        t_start_tot = 0
        for task in self.decision_variables["t_start"].keys():
            t_start_tot += self.decision_variables["t_start"][task]
            for task_pair in self.decision_variables["overlapping"].keys():
                if task in task_pair:
                    parallel_task = set(task_pair).difference({task}).pop()
                    task_type = tasks_type_correspondence[task]
                    parallel_task_type = tasks_type_correspondence[parallel_task]
                    overlapping = self.decision_variables["overlapping"][task_pair]
                    if (main_agent, task) in self.decision_variables["assignment"].keys() and (parallel_agent, parallel_task) in self.decision_variables["assignment"].keys():
                        assignment_main = self.decision_variables["assignment"][(main_agent, task)]
                        # assignment_parallel = self.decision_variables["assignment"][(parallel_agent, parallel_task)]
                        if (task_type, main_agent, parallel_task_type, parallel_agent) in tasks_synergies:
                            synergy = tasks_synergies[(task_type, main_agent, parallel_task_type, parallel_agent)]
                            sigma_val += overlapping * (synergy-1) * assignment_main * 10
                            sigma_tot += synergy
        self.model.addConstr(sigma_index == sigma_val/sigma_tot)

        J = self.model.addVar(name="J", vtype=gp.GRB.CONTINUOUS)
        self.model.addConstr(J == makespan + sigma_index + t_start_tot)
        # Objective function
        self.model.setObjective(J, gp.GRB.MINIMIZE)

    def add_overlapping_constraints(self, upper_bound=None):
        for task_i, task_j in self.decision_variables["overlapping"].keys():
            t_end_i = self.decision_variables["t_end"][task_i]
            t_end_j = self.decision_variables["t_end"][task_j]
            t_start_i = self.decision_variables["t_start"][task_i]
            t_start_j = self.decision_variables["t_start"][task_j]
            upper_bound = self.problem_definition.get_nominal_upper_bound() * 5

            raw_overlapping = self.model.addVar(name=f"raw_overlapping({task_i},{task_j})",
                                                vtype=gp.GRB.CONTINUOUS,
                                                lb=-upper_bound,
                                                ub=upper_bound)
            min_t_end = self.model.addVar(name=f"min_t_end({task_i}, {task_j})",
                                          vtype=gp.GRB.CONTINUOUS,
                                          lb=-upper_bound,
                                          ub=upper_bound)
            max_t_start = self.model.addVar(name=f"max_t_start({task_i}, {task_j})",
                                            vtype=gp.GRB.CONTINUOUS,
                                            lb=-upper_bound,
                                            ub=upper_bound)
            self.model.addConstr(min_t_end == gp.min_(t_end_i, t_end_j))
            self.model.addConstr(max_t_start == gp.max_(t_start_i, t_start_j))
            self.model.addConstr(raw_overlapping == (min_t_end - max_t_start))

            check_parallelism = self.model.addVar(name=f"check_parallelism({task_i},{task_j})",
                                                  vtype=gp.GRB.BINARY, lb=0, ub=1)

            self.model.addConstr(raw_overlapping >= -BIG_M * (1 - check_parallelism))
            self.model.addConstr(raw_overlapping <= BIG_M * check_parallelism - EPS)

            # self.model.addConstr((self.decision_variables["overlapping"][(task_i, task_j)] == (
            #         min_t_end - max_t_start) * check_parallelism))
            self.model.addConstr((self.decision_variables["overlapping"][(task_i, task_j)] == check_parallelism))
    # def add_t_end_constraints(self, agent_task_combination, cost) -> None:
    #     t_end = {}
    #     for agent, task in agent_task_combination:
    #         if task not in t_end:
    #             t_end[task] = self.decision_variables["t_start"][task]
    #         t_end[task] += self.decision_variables["assignment"][(agent, task)] * cost[(agent, task)]
        # tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
        # tasks_synergies = self.problem_definition.get_tasks_synergies()
        #
        # parallel_agent = "human_right_arm"
        # main_agent = "ur5_on_guide"
        # for task in self.decision_variables["t_start"].keys():
        #     for task_pair in self.decision_variables["overlapping"].keys():
        #         if task in task_pair:
        #             parallel_task = set(task_pair).difference({task}).pop()
        #             task_type = tasks_type_correspondence[task]
        #             parallel_task_type = tasks_type_correspondence[parallel_task]
        #             overlapping = self.decision_variables["overlapping"][task_pair]
        #             if (main_agent, task) in self.decision_variables["assignment"].keys():
        #                 assignment = self.decision_variables["assignment"][(main_agent, task)]
        #                 if (task_type, main_agent, parallel_task_type, parallel_agent) in tasks_synergies:
        #                     synergy = tasks_synergies[(task_type, main_agent, parallel_task_type, parallel_agent)]
        #                     t_end[task] += overlapping * (synergy - 1) * assignment
        #     self.model.addConstr(self.decision_variables["t_end"][task] == t_end[task], name=f"t_end({task})")
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

        for coppie in self.decision_variables["overlapping"].keys():
            print("-------------------------------------------------------")
            print(coppie)
            print(self.decision_variables["overlapping"][coppie].X)
            print("--------------------------------------------------------")
        for v in self.model.getVars():
            if "check_parallelism_geq_zero_" in v.varName:
                print(v.varName, v.x)
        return problem_solution
