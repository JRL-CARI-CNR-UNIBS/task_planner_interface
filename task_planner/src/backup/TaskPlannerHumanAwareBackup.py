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
class TaskPlannerHumanAware(TaskPlanner):
    def create_model(self) -> None:
        tasks_list = self.problem_definition.get_tasks_list()
        tasks_pairs = itertools.combinations(tasks_list, 2)
        upper_bound = self.problem_definition.get_nominal_upper_bound() * 10

        # self.decision_variables["overlapping"] = self.model.addVars(tasks_pairs,
        #                                                             name="overlapping",
        #                                                             vtype=gp.GRB.CONTINUOUS,
        #                                                             lb=0,
        #                                                             ub=self.problem_definition.get_nominal_upper_bound())
        self.decision_variables["overlapping"] = self.model.addVars(tasks_pairs,
                                                                    name="overlapping",
                                                                    vtype=gp.GRB.CONTINUOUS,
                                                                    lb=0,
                                                                    ub=upper_bound)
        super().create_model()
        self.add_overlapping_constraints()
        # self.model.setParam("MIPGapAbs", 0.28)

    def add_overlapping_constraints_test(self, upper_bound=None):
        parallel_agent = "human_right_arm"
        main_agent = "ur5_on_guide"

        # zero_var_main_i = self.model.addVar(name=f"zero_var_main_i",
        #                                     vtype=gp.GRB.BINARY,
        #                                     lb=0,
        #                                     ub=1)
        # zero_var_main_j = self.model.addVar(name=f"zero_var_main_j",
        #                                     vtype=gp.GRB.BINARY,
        #                                     lb=0,
        #                                     ub=1)
        # zero_var_para_i = self.model.addVar(name=f"zero_var_para_i",
        #                                     vtype=gp.GRB.BINARY,
        #                                     lb=0,
        #                                     ub=1)
        # zero_var_para_j = self.model.addVar(name=f"zero_var_para_j",
        #                                     vtype=gp.GRB.BINARY,
        #                                     lb=0,
        #                                     ub=1)
        #
        # self.model.addConstr(zero_var_main_i == 0)
        # self.model.addConstr(zero_var_main_j == 0)
        # self.model.addConstr(zero_var_para_i == 0)
        # self.model.addConstr(zero_var_para_j == 0)

        for task_i, task_j in self.decision_variables["overlapping"].keys():
            t_end_i = self.decision_variables["t_end"][task_i]
            t_end_j = self.decision_variables["t_end"][task_j]
            t_start_i = self.decision_variables["t_start"][task_i]
            t_start_j = self.decision_variables["t_start"][task_j]
            upper_bound = self.problem_definition.get_nominal_upper_bound()

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
            # aux = self.model.addVar(name=f"aux({task_i}, {task_j})",
            #                         vtype=gp.GRB.CONTINUOUS,
            #                         lb=0,
            #                         ub=upper_bound)
            # self.model.addConstr(aux == (min_t_end - max_t_start) * check_parallelism)
            # self.model.addConstr(self.decision_variables["overlapping"][(task_i, task_j)] == check_parallelism)
            self.model.addConstr((self.decision_variables["overlapping"][(task_i, task_j)] == (
                    min_t_end - max_t_start) * check_parallelism))
            # self.model.addConstr((check_ij == 0) >> self.decision_variables["overlapping"][(task_i, task_j)] == 0)

    def add_t_end_constraints(self, agent_task_combination, cost) -> None:
        t_end = {}
        for agent, task in agent_task_combination:
            if task not in t_end:
                t_end[task] = self.decision_variables["t_start"][task]
            t_end[task] += self.decision_variables["assignment"][(agent, task)] * cost[(agent, task)]
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
                    if (main_agent, task) in self.decision_variables["assignment"].keys():
                        assignment = self.decision_variables["assignment"][(main_agent, task)]
                        if (task_type, main_agent, parallel_task_type, parallel_agent) in tasks_synergies:
                            synergy = tasks_synergies[(task_type, main_agent, parallel_task_type, parallel_agent)]
                            t_end[task] += overlapping * (synergy - 1) * assignment
            self.model.addConstr(self.decision_variables["t_end"][task] == t_end[task], name=f"t_end({task})")
            # t_end = t_end[task] + gp.quicksum([self.decision_variables["overlapping"][task_pair]*(synergy_val())*self.decision_variables["assignment"][("ur5_on_guide",task)] for task_pair in self.decision_variables["overlapping"].keys() if task in task_pair])

    def add_overlapping_constraints(self, upper_bound=None):
        self.add_overlapping_constraints_test(upper_bound)

        # for task_i, task_j in self.decision_variables["overlapping"].keys():
        #     t_end_i = self.decision_variables["t_end"][task_i]
        #     t_end_j = self.decision_variables["t_end"][task_j]
        #     t_start_i = self.decision_variables["t_start"][task_i]
        #     t_start_j = self.decision_variables["t_start"][task_j]
        #     upper_bound = self.problem_definition.get_nominal_upper_bound()
        #
        #     raw_overlapping = self.model.addVar(name=f"raw_overlapping({task_i},{task_j})",
        #                                         vtype=gp.GRB.CONTINUOUS,
        #                                         lb=-upper_bound,
        #                                         ub=upper_bound)
        #     min_t_end = self.model.addVar(name=f"min_t_end({task_i}, {task_j})",
        #                                   vtype=gp.GRB.CONTINUOUS,
        #                                   lb=-upper_bound,
        #                                   ub=upper_bound)
        #     max_t_start = self.model.addVar(name=f"max_t_start({task_i}, {task_j})",
        #                                     vtype=gp.GRB.CONTINUOUS,
        #                                     lb=-upper_bound,
        #                                     ub=upper_bound)
        #     self.model.addConstr(min_t_end == gp.min_(t_end_i, t_end_j))
        #     self.model.addConstr(max_t_start == gp.max_(t_start_i, t_start_j))
        #     self.model.addConstr(raw_overlapping == (min_t_end - max_t_start))
        #
        #     check_parallelism = self.model.addVar(name=f"check_parallelism({task_i},{task_j})",
        #                                           vtype=gp.GRB.BINARY, lb=0, ub=1)
        #
        #     self.model.addConstr(raw_overlapping >= -BIG_M * (1 - check_parallelism))
        #     self.model.addConstr(raw_overlapping <= BIG_M * check_parallelism - EPS)
        #
        #     self.model.addConstr((self.decision_variables["overlapping"][(task_i, task_j)] == (
        #             min_t_end - max_t_start) * check_parallelism))

    def add_t_end_constraints_without_cost(self, agent_task_combination, cost) -> None:
        t_end = {}
        for agent, task in agent_task_combination:
            if task not in t_end:
                t_end[task] = self.decision_variables["t_start"][task]
            t_end[task] += self.decision_variables["assignment"][(agent, task)] * cost[(agent, task)]
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
                    if (main_agent, task) in self.decision_variables["assignment"].keys():
                        assignment = self.decision_variables["assignment"][(main_agent, task)]
                        if (task_type, main_agent, parallel_task_type, parallel_agent) in tasks_synergies:
                            synergy = tasks_synergies[(task_type, main_agent, parallel_task_type, parallel_agent)]
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

        for coppie in self.decision_variables["overlapping"].keys():
            print("-------------------------------------------------------")
            print(coppie)
            print(self.decision_variables["overlapping"][coppie].X)
            print("--------------------------------------------------------")
        for v in self.model.getVars():
            if "check_parallelism_geq_zero_" in v.varName:
                print(v.varName, v.x)
        return problem_solution

    def set_objective(self) -> None:
        agent_task_combination, cost = gp.multidict(self.problem_definition.get_combinations())
        makespan = self.model.addVar(name="makespan", vtype=gp.GRB.CONTINUOUS)
        # self.model.addConstr(makespan == gp.max_(self.decision_variables["t_end"]))
        self.model.addConstr(makespan == gp.quicksum(self.decision_variables["t_end"]) +
                             gp.quicksum(self.decision_variables["t_start"]))

        J = self.model.addVar(name="J", vtype=gp.GRB.CONTINUOUS)
        self.model.addConstr(J == makespan)
        # Objective function
        self.model.setObjective(J, gp.GRB.MINIMIZE)
