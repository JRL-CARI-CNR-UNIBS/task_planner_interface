# from Problem import Problem
# from Task import TaskSolution
# from TaskPlanner import TaskPlanner
#
# from dataclasses import dataclass, field
# from typing import List, Dict, Optional
#
# import gurobipy as gp
# import itertools
#
# from utils import Behaviour, Objective
#
# EPS = 1E-12
# BIG_M = 1E12
#
#
# @dataclass
# class Prove(TaskPlanner):
#     behaviour: Behaviour = field(default=Behaviour.CONTINUOUS)
#
#     # objective: Objective
#
#     def create_model(self) -> None:
#         tasks_list = self.problem_definition.get_tasks_list()
#         tasks_pairs = itertools.combinations(tasks_list, 2)
#         upper_bound = self.problem_definition.get_nominal_upper_bound()
#
#         if self.behaviour == Behaviour.DISCRETE:
#             self.decision_variables["overlapping"] = self.model.addVars(tasks_pairs,
#                                                                         name="overlapping",
#                                                                         vtype=gp.GRB.BINARY,
#                                                                         lb=0,
#                                                                         ub=1)
#         elif self.behaviour == Behaviour.CONTINUOUS:
#             self.decision_variables["overlapping"] = self.model.addVars(tasks_pairs,
#                                                                         name="overlapping",
#                                                                         vtype=gp.GRB.CONTINUOUS,
#                                                                         lb=0,
#                                                                         ub=15)
#         self.decision_variables["t_end_tilde"] = self.model.addVars(tasks_list,
#                                                               name="t_end_tilde",
#                                                               vtype=gp.GRB.CONTINUOUS,
#                                                               lb=0, ub=upper_bound*5)
#         super().create_model()
#         self.add_overlapping_constraints()
#         # self.model.setParam("cutoff",105)
#         # self.model.setParam("branchdir",-1)
#         # self.model.setParam("disconnected",2)
#         # self.model.setParam("NLPHeur", 1)
#         # self.model.setParam("Cuts", 3)
#         # self.model.setParam("MIPFocus", 3)
#
#         # self.model.setParam("heuristics", 0.001)
#         # self.model.setParam("MIPGapAbs", 0.28)
#         self.model.setParam("MIPGap", 0.297)
#
#     def add_general_constraints(self) -> None:
#         for couple_of_tasks, agent in self.decision_variables["delta_ij"]:
#             # Utility variable
#             task_i = couple_of_tasks[0]
#             task_j = couple_of_tasks[1]
#             agent = agent
#
#             # other_agent = {*self.agents}.difference({agent})
#             t_start_i = self.decision_variables["t_start"][task_i]
#             t_start_j = self.decision_variables["t_start"][task_j]
#
#             t_end_i = self.decision_variables["t_end_tilde"][task_i]  # t_start_i + cost[(agent,task_i)]
#             t_end_j = self.decision_variables["t_end_tilde"][task_j]  # t_start_i + cost[(agent,task_j)]
#
#             delta_ij = self.decision_variables["delta_ij"][((task_i, task_j), agent)]
#
#             assigned_both = self.model.addVar(name=f"both_{task_i}_{task_j}_to_{agent}", vtype=gp.GRB.BINARY)
#
#             # Constraints #
#             self.model.addConstr(assigned_both == gp.and_(self.decision_variables["assignment"][(agent, task_i)],
#                                                           self.decision_variables["assignment"][(agent, task_j)]))
#
#             # Delta ij constraint (check if t_start is greater or less...)
#             self.model.addConstr((assigned_both == 1) >> (t_start_i >= t_start_j + EPS - BIG_M * (1 - delta_ij)))
#             self.model.addConstr((assigned_both == 1) >> (t_start_i <= t_start_j + BIG_M * delta_ij))
#
#             # Not overlapping Constraints
#             self.model.addConstr((assigned_both == 1) >> (t_start_i >= t_end_j - BIG_M * (1 - delta_ij)))
#             self.model.addConstr((assigned_both == 1) >> (t_start_j >= t_end_i - BIG_M * delta_ij))
#
#     def add_t_end_constraints(self, agent_task_combination, cost) -> None:
#         t_end = {}
#         for agent, task in agent_task_combination:
#             if task not in t_end:
#                 t_end[task] = self.decision_variables["t_start"][task]
#             t_end[task] += self.decision_variables["assignment"][(agent, task)] * cost[(agent, task)]
#         tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
#         tasks_synergies = self.problem_definition.get_tasks_synergies()
#
#         parallel_agent = "human_right_arm"
#         main_agent = "ur5_on_guide"
#         # TODO: define agents in "init"
#         for task in self.decision_variables["t_start"].keys():
#             self.model.addConstr(self.decision_variables["t_end"][task] == t_end[task], name=f"t_end({task})")
#             for task_pair in self.decision_variables["overlapping"].keys():
#                 if task in task_pair:
#                     parallel_task = set(task_pair).difference({task}).pop()
#                     task_type = tasks_type_correspondence[task]
#                     parallel_task_type = tasks_type_correspondence[parallel_task]
#                     overlapping = self.decision_variables["overlapping"][task_pair]
#                     if (main_agent, task) in self.decision_variables["assignment"].keys():
#                         assignment_main_task = self.decision_variables["assignment"][(main_agent, task)]
#                         if (task_type, main_agent, parallel_task_type, parallel_agent) in tasks_synergies:
#                             synergy = tasks_synergies[(task_type, main_agent, parallel_task_type, parallel_agent)]
#                             if self.behaviour == Behaviour.CONTINUOUS:
#                                 t_end[task] += overlapping * (synergy - 1) * assignment_main_task
#                             elif self.behaviour == Behaviour.DISCRETE:  # Add cost (Measurement: time)
#                                 t_end[task] += overlapping * (synergy - 1) * assignment_main_task * cost[
#                                     (main_agent, task)]
#             self.model.addConstr(self.decision_variables["t_end_tilde"][task] == t_end[task], name=f"t_end({task})")
#
#             # TODO: Pensare a cosa simile a quella sotto: aggiungere vincolo = t_end MODIFICATO SOLO se parallel task è assegnato alla persona.
#             # Note: Così come commentato non è corretto perchè nel caso non lo fosse dovrebbe essere uguale a t_start + d*a.
#             # Bisognerebbe fare vincolo sopra e poi aggiungerlo sotto, ma si riferirà allo stesso vincolo o agigunge un'altro?
#             # self.model.addConstr(self.decision_variables["overlapping"][parallel_task] == 1 >>
#             #                      (self.decision_variables["t_end"][task] == t_end[task]),
#             #                      name=f"t_end({task})")
#
#     def add_overlapping_constraints(self, upper_bound=None):
#
#         for task_i, task_j in self.decision_variables["overlapping"].keys():
#             t_end_i = self.decision_variables["t_end"][task_i]
#             t_end_j = self.decision_variables["t_end"][task_j]
#             t_start_i = self.decision_variables["t_start"][task_i]
#             t_start_j = self.decision_variables["t_start"][task_j]
#             upper_bound = self.problem_definition.get_nominal_upper_bound()
#             # TODO: Remove from here upper bound
#             raw_overlapping = self.model.addVar(name=f"raw_overlapping({task_i},{task_j})",
#                                                 vtype=gp.GRB.CONTINUOUS,
#                                                 lb=-upper_bound,
#                                                 ub=upper_bound)
#             min_t_end = self.model.addVar(name=f"min_t_end({task_i}, {task_j})",
#                                           vtype=gp.GRB.CONTINUOUS,
#                                           lb=0,
#                                           ub=upper_bound)
#             max_t_start = self.model.addVar(name=f"max_t_start({task_i}, {task_j})",
#                                             vtype=gp.GRB.CONTINUOUS,
#                                             lb=0,
#                                             ub=upper_bound)
#
#             # aux_min_t_end = self.model.addVar(name=f"min_aux({task_i}, {task_j})",
#             #                                   vtype=gp.GRB.BINARY,
#             #                                   lb=0,
#             #                                   ub=1)
#             # aux_max_t_start = self.model.addVar(name=f"max_aux({task_i}, {task_j})",
#             #                                     vtype=gp.GRB.BINARY,
#             #                                     lb=0,
#             #                                     ub=1)
#             # self.model.addConstr(min_t_end == gp.min_(t_end_i, t_end_j))
#             # self.model.addConstr(max_t_start == gp.max_(t_start_i, t_start_j))
#             # self.model.addConstr(min_t_end <= t_end_i)
#             # self.model.addConstr(min_t_end <= t_end_j)
#             # self.model.addConstr(min_t_end >= t_end_i - BIG_M * (1 - aux_min_t_end))
#             # self.model.addConstr(min_t_end >= t_end_j - BIG_M * aux_min_t_end)
#             #
#             # self.model.addConstr(max_t_start >= t_start_i)
#             # self.model.addConstr(max_t_start >= t_start_j)
#             # self.model.addConstr(max_t_start <= t_start_i + BIG_M * aux_max_t_start)
#             # self.model.addConstr(max_t_start <= t_start_j + BIG_M * (1 - aux_max_t_start))
#             self.model.addGenConstrMin(min_t_end, [t_end_i, t_end_j])
#             self.model.addGenConstrMax(max_t_start, [t_start_i, t_start_j])
#
#             self.model.addConstr(raw_overlapping == (min_t_end - max_t_start))
#
#             check_parallelism = self.model.addVar(name=f"check_parallelism({task_i},{task_j})",
#                                                   vtype=gp.GRB.BINARY, lb=0, ub=1)
#
#             self.model.addConstr(raw_overlapping >= -BIG_M * (1 - check_parallelism))
#             self.model.addConstr(raw_overlapping <= BIG_M * check_parallelism - EPS)
#
#             if self.behaviour == Behaviour.DISCRETE:
#                 self.model.addConstr((self.decision_variables["overlapping"][(task_i, task_j)] == check_parallelism))
#             elif self.behaviour == Behaviour.CONTINUOUS:
#                 self.model.addConstr((self.decision_variables["overlapping"][(task_i, task_j)] == (
#                         min_t_end - max_t_start) * check_parallelism))
#
#     # def get_solution(self) -> List[TaskSolution]:
#     #     for v in self.model.getVars():
#     #         if "idle" in v.varName or "min_tend" in v.varName or "duration" in v.varName or "tStart" in v.varName or "assignment" in v.varName or "tot" in v.varName or "t_end_robot" in v.varName or "t_end_human" in v.varName or "t_end" in v.varName or "t_start" in v.varName:
#     #             print(v.varName, v.x)
#     #
#     #     agents = self.problem_definition.get_agents()
#     #     task_lists = self.problem_definition.get_tasks_list()
#     #
#     #     problem_solution = []
#     #     for task in task_lists:
#     #         t_start = self.decision_variables["t_start"][task].X
#     #         t_end = self.decision_variables["t_end"][task].X
#     #         # assignments = self.decision_variables["assignment"].select("*", task)
#     #
#     #         assignment = None
#     #         for agent in agents:
#     #             decision_variable = self.decision_variables["assignment"].select(agent, task)
#     #             if decision_variable:
#     #                 assert len(decision_variable) == 1
#     #                 if len(decision_variable) == 1 and decision_variable[0].X == 1:
#     #                     assignment = agent
#     #         print(task, t_start, t_end, assignment)
#     #         try:
#     #             task_solution = self.problem_definition.add_task_solution(task, t_start, t_end, assignment)
#     #             print(task_solution)
#     #         except ValueError:
#     #             print(f"Error during solution filling")
#     #             raise Exception(f"T start of task: {task}, is negative: {t_start}, t_end: {t_end}, by: {assignment}")
#     #         except Exception:
#     #             print(f"Error during solution filling")
#     #             raise Exception(f"{task}, not presence in problem task list")
#     #         problem_solution.append(task_solution)
#     #     return problem_solution
#
#     def set_objective(self) -> None:
#         # TODO: Can be put in TaskPlanner Base class.
#         cost_function = self.model.addVar(name="J", vtype=gp.GRB.CONTINUOUS, lb = -100)
#
#         if self.objective == Objective.SUM_T_START_END:
#             self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["t_end"]) +
#                                  gp.quicksum(self.decision_variables["t_start"]))
#         elif self.objective == Objective.MAKESPAN:
#             # task_list = [task for task in self.decision_variables["t_start"].keys()]
#             # upper_bound = self.problem_definition.get_nominal_upper_bound()
#             # aux_obj_max = self.model.addVars(self.decision_variables["t_start"].keys(),
#             #                                  name="aux_obj_max",
#             #                                  vtype=gp.GRB.CONTINUOUS,
#             #                                  lb=0,
#             #                                  ub=upper_bound)
#             # for task in task_list:
#             #     self.model.addConstr(cost_function >= self.decision_variables["t_end"][task])
#             #     self.model.addConstr(cost_function <= self.decision_variables["t_end"][task] + (1-aux_obj_max[task])* BIG_M)
#             # self.model.addConstr(gp.quicksum(aux_obj_max) == 1)
#             #     #
#             # self.model.addConstr(cost_function == gp.max_(self.decision_variables["t_end"]))
#             self.model.addGenConstrMax(cost_function, self.decision_variables["t_end_tilde"])
#         elif self.objective == Objective.SUM_T_START:
#             self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["t_start"]))
#         elif self.objective == Objective.SUM_T_END:
#             self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["t_end"]))
#         elif self.objective == Objective.SYNERGY:
#             upper_bound = self.problem_definition.get_nominal_upper_bound()
#             parallel_agent = "human_right_arm"
#             main_agent = "ur5_on_guide"
#             t_start_robot = self.model.addVars(list(self.decision_variables["t_start"].keys()), lb=0, ub=upper_bound,
#                                                name="t_start_robot", vtype=gp.GRB.CONTINUOUS)
#             t_start_human = self.model.addVars(list(self.decision_variables["t_start"].keys()), lb=0, ub=upper_bound,
#                                                name="t_start_human", vtype=gp.GRB.CONTINUOUS)
#             t_end_robot = self.model.addVars(list(self.decision_variables["t_end"].keys()), lb=0, ub=upper_bound,
#                                              name="t_end_robot", vtype=gp.GRB.CONTINUOUS)
#             t_end_human = self.model.addVars(list(self.decision_variables["t_end"].keys()), lb=0, ub=upper_bound,
#                                              name="t_end_human", vtype=gp.GRB.CONTINUOUS)
#             duration_human = self.model.addVar(lb=0, ub=upper_bound*10, name="duration_human", vtype=gp.GRB.CONTINUOUS)
#             duration_robot = self.model.addVar(lb=0, ub=upper_bound*10, name="duration_human", vtype=gp.GRB.CONTINUOUS)
#
#             for task in self.decision_variables["t_start"].keys():
#                 print(task)
#                 if (main_agent, task) in self.decision_variables["assignment"].keys():
#                     print(main_agent, task)
#                     self.model.addConstr(t_start_robot[task] == self.decision_variables["assignment"][(main_agent, task)] * self.decision_variables["t_start"][task])
#                     self.model.addConstr(t_end_robot[task] == self.decision_variables["assignment"][(main_agent, task)] * self.decision_variables["t_end_tilde"][task])
#                 else:
#                     self.model.addConstr(t_start_robot[task] == 0)
#                     self.model.addConstr(t_end_robot[task] == 0)
#                 if (parallel_agent, task) in self.decision_variables["assignment"].keys():
#                     print(parallel_agent, task)
#                     self.model.addConstr(
#                         t_start_human[task] == self.decision_variables["assignment"][(parallel_agent, task)] * self.decision_variables["t_start"][task])
#                     self.model.addConstr(t_end_human[task] == self.decision_variables["assignment"][(parallel_agent, task)] * self.decision_variables["t_end_tilde"][task])
#                 else:
#                     self.model.addConstr(t_start_human[task] == 0)
#                     self.model.addConstr(t_end_human[task] == 0)
#             print(t_start_human.keys())
#             print(t_end_human.keys())
#             print(t_start_robot.keys())
#             print(t_end_robot.keys())
#             self.model.addConstr(duration_robot == gp.max_(t_end_robot))
#             self.model.addConstr(duration_human == gp.max_(t_end_human))
#
#             tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
#             tasks_synergies = self.problem_definition.get_tasks_synergies()
#
#             sigma_val = 0.0
#             sigma_tot = 0.0
#             _, cost = gp.multidict(self.problem_definition.get_combinations())
#             for task in self.decision_variables["t_start"].keys():
#                 for task_pair in self.decision_variables["overlapping"].keys():
#                     if task in task_pair:
#                         parallel_task = set(task_pair).difference({task}).pop()
#                         task_type = tasks_type_correspondence[task]
#                         parallel_task_type = tasks_type_correspondence[parallel_task]
#                         overlapping = self.decision_variables["overlapping"][task_pair]
#                         if (main_agent, task) in self.decision_variables["assignment"].keys() and \
#                                 (parallel_agent, parallel_task) in self.decision_variables["assignment"].keys():
#                             assignment_main = self.decision_variables["assignment"][(main_agent, task)]
#                             if (task_type, main_agent, parallel_task_type, parallel_agent) in tasks_synergies:
#                                 synergy = tasks_synergies[(task_type, main_agent, parallel_task_type, parallel_agent)]
#                                 if self.behaviour == Behaviour.CONTINUOUS:
#                                     sigma_val += overlapping * (synergy - 1) * assignment_main
#                                 elif self.behaviour == Behaviour.DISCRETE:
#                                     sigma_val += overlapping * (synergy - 1) * assignment_main * cost[
#                                         (main_agent, task)]
#                                 # sigma_tot += synergy
#             # duration = self.model.addVar(lb=0,
#             #                              ub=self.problem_definition.get_nominal_upper_bound()*5,
#             #                              name="duration",
#             #                              vtype=gp.GRB.CONTINUOUS)
#             idle_human = self.model.addVar(name="idle_human",lb=0,ub=self.problem_definition.get_nominal_upper_bound()*5,vtype=gp.GRB.CONTINUOUS)
#             idle_robot = self.model.addVar(name="idle_robot",lb=0,ub=self.problem_definition.get_nominal_upper_bound()*5,vtype=gp.GRB.CONTINUOUS)
#
#             # self.model.addConstr(duration == gp.max_(self.decision_variables["t_end"]))
#             # self.model.addConstr(cost_function >= (-3 +
#             #                                        sigma_val + duration + duration - gp.quicksum(
#             #             self.decision_variables["t_end"]) + gp.quicksum(
#             #             self.decision_variables["t_start"])))
#             # self.model.addConstr(cost_function <= (3 +
#             #                                        sigma_val + duration + duration - gp.quicksum(
#             #             self.decision_variables["t_end"]) + gp.quicksum(
#             #             self.decision_variables["t_start"])))
#             # self.model.addConstr(cost_function == (duration_robot + duration_human +
#             #                                        sigma_val - gp.quicksum(t_end_robot) - gp.quicksum(t_end_human)
#             #                                        + gp.quicksum(t_start_robot) + gp.quicksum(t_start_human)))
#             self.model.addConstr(idle_robot == duration_robot - gp.quicksum(t_end_robot) + gp.quicksum(t_start_robot))
#             self.model.addConstr(idle_human == duration_human - gp.quicksum(t_end_human) + gp.quicksum(t_start_human))
#             self.model.addConstr(cost_function == sigma_val + idle_human + idle_robot)
#
#             # self.model.addConstr(duration == gp.max_(self.decision_variables["t_end"]))
#
#             # self.model.addConstr(duration <= 95)
#
#         # Objective function
#         self.model.setObjective(cost_function, gp.GRB.MINIMIZE)
#
#         self.model.write("Model.lp")
#         # self.model.tune()
#
#
from Problem import Problem
from Task import TaskSolution
from TaskPlanner import TaskPlanner

from dataclasses import dataclass, field
from typing import List, Dict, Optional

import gurobipy as gp
import itertools

from utils import *

EPS = 1E-12
BIG_M = 1E12


@dataclass
class Prove(TaskPlanner):
    behaviour: Behaviour = field(default=Behaviour.CONTINUOUS)

    # objective: Objective

    def create_model(self) -> None:
        tasks_list = self.problem_definition.get_tasks_list()
        tasks_pairs = itertools.combinations(tasks_list, 2)
        upper_bound = self.problem_definition.get_nominal_upper_bound()
        self.decision_variables["duration"] = self.model.addVars(tasks_list,
                                                                 name="duration",
                                                                 vtype=gp.GRB.CONTINUOUS,
                                                                 lb=0,
                                                                 ub=self.problem_definition.get_nominal_upper_bound())
        self.decision_variables["d_i_r_tilde"] = self.model.addVars(tasks_list,
                                                                    name="d_i_r_tilde",
                                                                    vtype=gp.GRB.CONTINUOUS,
                                                                    lb=0,
                                                                    ub=self.problem_definition.get_nominal_upper_bound())
        if self.behaviour == Behaviour.DISCRETE:
            self.decision_variables["overlapping"] = self.model.addVars(tasks_pairs,
                                                                        name="overlapping",
                                                                        vtype=gp.GRB.BINARY,
                                                                        lb=0,
                                                                        ub=1)
        elif self.behaviour == Behaviour.CONTINUOUS:
            self.decision_variables["overlapping"] = self.model.addVars(tasks_pairs,
                                                                        name="overlapping",
                                                                        vtype=gp.GRB.CONTINUOUS,
                                                                        lb=0,
                                                                        ub=self.problem_definition.get_max_overlapping())
        # self.decision_variables["t_end_tilde"] = self.model.addVars(tasks_list,
        #                                                             name="t_end_tilde",
        #                                                             vtype=gp.GRB.CONTINUOUS,
        #                                                             lb=0, ub=upper_bound * 5)
        super().create_model()

        self.add_overlapping_constraints()
        # self.model.setParam("MIPGap", 0.201)

    def add_general_constraints(self) -> None:
        for couple_of_tasks, agent in self.decision_variables["delta_ij"]:
            # Utility variable
            task_i = couple_of_tasks[0]
            task_j = couple_of_tasks[1]
            agent = agent

            # other_agent = {*self.agents}.difference({agent})
            t_start_i = self.decision_variables["t_start"][task_i]
            t_start_j = self.decision_variables["t_start"][task_j]

            t_end_i = self.decision_variables["t_end"][task_i]  # t_start_i + cost[(agent,task_i)]
            t_end_j = self.decision_variables["t_end"][task_j]  # t_start_i + cost[(agent,task_j)]

            delta_ij = self.decision_variables["delta_ij"][((task_i, task_j), agent)]

            assigned_both = self.model.addVar(name=f"both_{task_i}_{task_j}_to_{agent}", vtype=gp.GRB.BINARY)

            # Constraints #
            self.model.addConstr(assigned_both == gp.and_(self.decision_variables["assignment"][(agent, task_i)],
                                                          self.decision_variables["assignment"][(agent, task_j)]))

            # Delta ij constraint (check if t_start is greater or less...)
            self.model.addConstr((assigned_both == 1) >> (t_start_i >= t_start_j + EPS - BIG_M * (1 - delta_ij)))
            self.model.addConstr((assigned_both == 1) >> (t_start_i <= t_start_j + BIG_M * delta_ij))

            # Not overlapping Constraints
            self.model.addConstr((assigned_both == 1) >> (t_start_i >= t_end_j - BIG_M * (1 - delta_ij)))
            self.model.addConstr((assigned_both == 1) >> (t_start_j >= t_end_i - BIG_M * delta_ij))

    def add_t_end_constraints(self, agent_task_combination, cost) -> None:
        # self.add_t_end_constraints2(agent_task_combination, cost)
        agent_h = "human_right_arm"
        agent_r = "ur5_on_guide"
        tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
        tasks_synergies = self.problem_definition.get_tasks_synergies()

        d_i_r_tilde = {}
        for task_i in self.decision_variables["t_start"].keys():

            if (agent_h, task_i) not in self.decision_variables["assignment"].keys():
                duration_i_h = 0
            else:
                a_i_h = self.decision_variables["assignment"][(agent_h, task_i)]
                d_i_h_hat = cost[(agent_h, task_i)]
                duration_i_h = a_i_h * d_i_h_hat

            if (agent_r, task_i) not in self.decision_variables["assignment"].keys():
                duration_i_r = 0
            else:
                if task_i not in d_i_r_tilde:
                    d_i_r_tilde[task_i] = cost[(agent_r, task_i)]
                for task_pair_ik in self.decision_variables["overlapping"].keys():
                    if task_i in task_pair_ik:
                        task_k = set(task_pair_ik).difference({task_i}).pop()
                        task_type_i = tasks_type_correspondence[task_i]
                        task_type_k = tasks_type_correspondence[task_k]
                        overlapping = self.decision_variables["overlapping"][task_pair_ik]
                        if (task_type_i, agent_r, task_type_k, agent_h) in tasks_synergies:
                            synergy = tasks_synergies[(task_type_i, agent_r, task_type_k, agent_h)]
                            if (agent_h, task_k) in self.decision_variables["assignment"]:
                                a_k_h = self.decision_variables["assignment"][(agent_h, task_k)]
                                d_i_r_tilde[task_i] += overlapping * (synergy - 1) * a_k_h
                self.model.addConstr(self.decision_variables["d_i_r_tilde"][task_i] == d_i_r_tilde[task_i])
                self.model.addConstr(self.decision_variables["d_i_r_tilde"][task_i] >= 0.7 * cost[(agent_r, task_i)])
                duration_i_r = self.decision_variables["d_i_r_tilde"][task_i] * self.decision_variables["assignment"][
                    (agent_r, task_i)]
            d_i = self.decision_variables["duration"][task_i]

            self.model.addConstr(d_i == duration_i_h + duration_i_r)

            self.model.addConstr(
                self.decision_variables["duration"][task_i] == self.decision_variables["t_end"][task_i] -
                self.decision_variables["t_start"][task_i])

    # def add_t_end_constraints2(self,
    #                            agent_task_combination,
    #                            cost) -> None:
    #     # Tend constraints
    #     t_end = {}
    #     print(agent_task_combination)
    #     for agent, task in agent_task_combination:
    #         if task not in t_end:
    #             t_end[task] = self.decision_variables["t_start"][task]
    #         t_end[task] += self.decision_variables["assignment"][(agent, task)] * cost[(agent, task)]
    #
    #     [self.model.addConstr(
    #         self.decision_variables["t_end"][task] == t_end_value) for task, t_end_value in t_end.items()]

    def add_overlapping_constraints(self, upper_bound=None):

        for task_i, task_k in self.decision_variables["overlapping"].keys():
            d_i = self.decision_variables["t_end"][task_i] - self.decision_variables["t_start"][task_i]
            d_k = self.decision_variables["t_end"][task_k] - self.decision_variables["t_start"][task_k]
            # self.model.addConstr(self.decision_variables["duration"][task_i] == d_i)
            # self.model.addConstr(self.decision_variables["duration"][task_k] == d_k)

            t_i_start = self.decision_variables["t_start"][task_i]
            t_i_end = self.decision_variables["t_end"][task_i]
            t_k_start = self.decision_variables["t_start"][task_k]
            t_k_end = self.decision_variables["t_end"][task_k]

            # check_parallelism = 1 -> overlapping parts
            check_parallelism = self.model.addVar(name=f"check_parallelism({task_i},{task_k})",
                                                  vtype=gp.GRB.BINARY, lb=0, ub=1)

            not_overlapped_parts_sup = self.model.addVar(name=f"not_overlapped_parts_sup({task_i},{task_k})",
                                                         vtype=gp.GRB.CONTINUOUS,
                                                         lb=0,
                                                         ub=self.problem_definition.get_nominal_upper_bound())
            not_overlapped_parts_inf = self.model.addVar(name=f"not_overlapped_parts_inf({task_i},{task_k})",
                                                         vtype=gp.GRB.CONTINUOUS,
                                                         lb=0,
                                                         ub=self.problem_definition.get_nominal_upper_bound())
            delta_end_ik = self.model.addVar(name=f"delta_end_ik({task_i},{task_k})",
                                             vtype=gp.GRB.CONTINUOUS,
                                             lb=-self.problem_definition.get_nominal_upper_bound(),
                                             ub=self.problem_definition.get_nominal_upper_bound())
            delta_start_ik = self.model.addVar(name=f"delta_start_ik({task_i},{task_k})",
                                               vtype=gp.GRB.CONTINUOUS,
                                               lb=-self.problem_definition.get_nominal_upper_bound(),
                                               ub=self.problem_definition.get_nominal_upper_bound())

            sum_delta_ik = self.model.addVar(name=f"sum_delta_ik({task_i},{task_k})",
                                             vtype=gp.GRB.CONTINUOUS,
                                             lb=-self.problem_definition.get_nominal_upper_bound(),
                                             ub=self.problem_definition.get_nominal_upper_bound())
            abs_sum_delta_ik = self.model.addVar(name=f"abs_sum_delta_ik({task_i},{task_k})",
                                                 vtype=gp.GRB.CONTINUOUS,
                                                 lb=0,
                                                 ub=self.problem_definition.get_nominal_upper_bound())
            self.model.addConstr(delta_end_ik == t_i_end - t_k_end)
            self.model.addConstr(delta_start_ik == t_i_start - t_k_start)

            self.model.addConstr(not_overlapped_parts_sup == gp.abs_(delta_end_ik))
            self.model.addConstr(not_overlapped_parts_inf == gp.abs_(delta_start_ik))

            self.model.addConstr(d_i + d_k >= not_overlapped_parts_sup + not_overlapped_parts_inf + EPS - BIG_M * (
                    1 - check_parallelism))
            self.model.addConstr(
                d_i + d_k <= not_overlapped_parts_sup + not_overlapped_parts_inf + BIG_M * check_parallelism)

            self.model.addConstr(sum_delta_ik == delta_end_ik + delta_start_ik)
            ov_ik = self.decision_variables["overlapping"][(task_i, task_k)]
            self.model.addConstr(abs_sum_delta_ik == gp.abs_(sum_delta_ik))
            self.model.addConstr(
                (check_parallelism == 1) >> (2 * ov_ik <= d_i + d_k - abs_sum_delta_ik))
            self.model.addConstr((check_parallelism == 0) >> (ov_ik == 0))

    def set_objective(self) -> None:
        # TODO: Can be put in TaskPlanner Base class.
        cost_function = self.model.addVar(name="J", vtype=gp.GRB.CONTINUOUS, lb=-100)

        if self.objective == Objective.SUM_T_START_END:
            self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["t_end"]) +
                                 gp.quicksum(self.decision_variables["t_start"]))
        elif self.objective == Objective.MAKESPAN:
            self.model.addGenConstrMax(cost_function, self.decision_variables["t_end_tilde"])
        elif self.objective == Objective.SUM_T_START:
            self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["t_start"]))
        elif self.objective == Objective.SUM_T_END:
            self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["t_end"]))
        elif self.objective == Objective.SYNERGY:
            upper_bound = self.problem_definition.get_nominal_upper_bound()
            parallel_agent = "human_right_arm"
            main_agent = "ur5_on_guide"
            t_start_robot = self.model.addVars(list(self.decision_variables["t_start"].keys()), lb=0, ub=upper_bound,
                                               name="t_start_robot", vtype=gp.GRB.CONTINUOUS)
            t_start_human = self.model.addVars(list(self.decision_variables["t_start"].keys()), lb=0, ub=upper_bound,
                                               name="t_start_human", vtype=gp.GRB.CONTINUOUS)
            t_end_robot = self.model.addVars(list(self.decision_variables["t_end"].keys()), lb=0, ub=upper_bound,
                                             name="t_end_robot", vtype=gp.GRB.CONTINUOUS)
            t_end_human = self.model.addVars(list(self.decision_variables["t_end"].keys()), lb=0, ub=upper_bound,
                                             name="t_end_human", vtype=gp.GRB.CONTINUOUS)
            duration_human = self.model.addVar(lb=0, ub=upper_bound * 10, name="duration_human",
                                               vtype=gp.GRB.CONTINUOUS)
            duration_robot = self.model.addVar(lb=0, ub=upper_bound * 10, name="duration_human",
                                               vtype=gp.GRB.CONTINUOUS)

            for task in self.decision_variables["t_start"].keys():
                print(task)
                if (main_agent, task) in self.decision_variables["assignment"].keys():
                    print(main_agent, task)
                    self.model.addConstr(
                        t_start_robot[task] == self.decision_variables["assignment"][(main_agent, task)] *
                        self.decision_variables["t_start"][task])
                    self.model.addConstr(
                        t_end_robot[task] == self.decision_variables["assignment"][(main_agent, task)] *
                        self.decision_variables["t_end"][task])
                else:
                    self.model.addConstr(t_start_robot[task] == 0)
                    self.model.addConstr(t_end_robot[task] == 0)
                if (parallel_agent, task) in self.decision_variables["assignment"].keys():
                    print(parallel_agent, task)
                    self.model.addConstr(
                        t_start_human[task] == self.decision_variables["assignment"][(parallel_agent, task)] *
                        self.decision_variables["t_start"][task])
                    self.model.addConstr(
                        t_end_human[task] == self.decision_variables["assignment"][(parallel_agent, task)] *
                        self.decision_variables["t_end"][task])
                else:
                    self.model.addConstr(t_start_human[task] == 0)
                    self.model.addConstr(t_end_human[task] == 0)
            print(t_start_human.keys())
            print(t_end_human.keys())
            print(t_start_robot.keys())
            print(t_end_robot.keys())
            self.model.addConstr(duration_robot == gp.max_(t_end_robot))
            self.model.addConstr(duration_human == gp.max_(t_end_human))

            tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
            tasks_synergies = self.problem_definition.get_tasks_synergies()

            sigma_val = 0.0
            sigma_tot = 0.0
            self.sigma_var = self.model.addVar(name="sigma_var", vtype=gp.GRB.CONTINUOUS, lb=-100)
            self.durata = self.model.addVar(name="durata", vtype=gp.GRB.CONTINUOUS, lb=0)

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
                            if (task_type, main_agent, parallel_task_type, parallel_agent) in tasks_synergies:
                                synergy = tasks_synergies[(task_type, main_agent, parallel_task_type, parallel_agent)]
                                if self.behaviour == Behaviour.CONTINUOUS:
                                    sigma_val += overlapping * (synergy - 1) * assignment_main
                                elif self.behaviour == Behaviour.DISCRETE:
                                    sigma_val += overlapping * (synergy - 1) * assignment_main * cost[
                                        (main_agent, task)]
                                # sigma_tot += synergy
            # duration = self.model.addVar(lb=0,
            #                              ub=self.problem_definition.get_nominal_upper_bound()*5,
            #                              name="duration",
            #                              vtype=gp.GRB.CONTINUOUS)
            idle_human = self.model.addVar(name="idle_human", lb=0,
                                           ub=self.problem_definition.get_nominal_upper_bound() * 5,
                                           vtype=gp.GRB.CONTINUOUS)
            idle_robot = self.model.addVar(name="idle_robot", lb=0,
                                           ub=self.problem_definition.get_nominal_upper_bound() * 5,
                                           vtype=gp.GRB.CONTINUOUS)

            # self.model.addConstr(duration == gp.max_(self.decision_variables["t_end"]))
            # self.model.addConstr(cost_function >= (-3 +
            #                                        sigma_val + duration + duration - gp.quicksum(
            #             self.decision_variables["t_end"]) + gp.quicksum(
            #             self.decision_variables["t_start"])))
            # self.model.addConstr(cost_function <= (3 +
            #                                        sigma_val + duration + duration - gp.quicksum(
            #             self.decision_variables["t_end"]) + gp.quicksum(
            #             self.decision_variables["t_start"])))
            # self.model.addConstr(cost_function == (duration_robot + duration_human +
            #                                        sigma_val - gp.quicksum(t_end_robot) - gp.quicksum(t_end_human)
            #                                        + gp.quicksum(t_start_robot) + gp.quicksum(t_start_human)))
            self.model.addConstr(idle_robot == duration_robot - gp.quicksum(t_end_robot) + gp.quicksum(t_start_robot))
            self.model.addConstr(idle_human == duration_human - gp.quicksum(t_end_human) + gp.quicksum(t_start_human))
            self.model.addConstr(self.durata == gp.max_(self.decision_variables["t_end"]))
            self.model.addConstr(self.sigma_var == sigma_val)

            self.model.addConstr(cost_function == self.durata + self.sigma_var)

            # self.model.addConstr(cost_function == sigma_val )

            # print(self.model.getVars())

            # self.model.addConstr(duration == gp.max_(self.decision_variables["t_end"]))

            # self.model.addConstr(duration <= 95)

        else:
            self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["d_i_r_tilde"]))

        # Objective function
        self.model.setObjective(cost_function, gp.GRB.MINIMIZE)
        #
        # self.model.write("Model.lp")
        # self.model.tune()

    def solve(self) -> None:
        # Optimization
        self.model.params.NonConvex = 2
        # self.model.optimize()
        self.model.optimize(lambda model, where: self.callback(model, where))

        status = self.model.Status
        if status in (gp.GRB.INF_OR_UNBD, gp.GRB.INFEASIBLE, gp.GRB.UNBOUNDED):
            print('The model cannot be solved because it is infeasible or '
                  'unbounded')
            return False
        if status != gp.GRB.OPTIMAL:
            print('Optimization was stopped with status ' + str(status))
            return False
        return True

    def callback(self, model, where):
        if where == gp.GRB.Callback.MIPSOL:
            print(f"Sigma index: {model.cbGetSolution(self.sigma_var)}")
            print(f"Makespan: {model.cbGetSolution(self.durata)}")
            agents = self.problem_definition.get_agents()
            task_lists = self.problem_definition.get_tasks_list()

            problem_solution = []
            for task in task_lists:
                # print(self.decision_variables["t_start"][task])
                t_start = model.cbGetSolution(self.decision_variables["t_start"][task])
                t_end = model.cbGetSolution(self.decision_variables["t_end"][task])
                # print(t_start)
                #         # assignments = self.decision_variables["assignment"].select("*", task)
                #
                assignment = None
                for agent in agents:
                    # print()
                    if (agent, task) in self.decision_variables["assignment"].keys():
                        # print(self.decision_variables["assignment"][(agent, task)])
                        decision_variable = model.cbGetSolution(self.decision_variables["assignment"][(agent, task)])
                        if decision_variable == 1:
                            # if decision_variable:
                            #     assert len(decision_variable) == 1
                            #     if len(decision_variable) == 1 and decision_variable[0].X == 1:
                            assignment = agent
                #         # print(task, t_start, t_end, assignment)
                try:
                    task_solution = self.problem_definition.add_task_solution(task, t_start, t_end, assignment)
                except ValueError:
                    print(f"Error during solution filling")
                    raise Exception(
                        f"T start of task: {task}, is negative: {t_start}, t_end: {t_end}, by: {assignment}")
                except Exception:
                    print(f"Error during solution filling")
                    raise Exception(f"{task}, not presence in problem task list")
                problem_solution.append(task_solution)
            # print(problem_solution)
            show_timeline(problem_solution)

            problem_solution.sort(key=lambda task_sol: task_sol.get_start_time())

            problem_solution_agent = {agent: list(filter(lambda task_solution: task_solution.get_assignment()
                                                                               == agent, problem_solution)) for agent in
                                      ["ur5_on_guide", "human_right_arm"]}

            for main_agent_task_sol in problem_solution_agent["ur5_on_guide"]:
                main_agent_task_sol: TaskSolution
                # print(main_agent_task_sol.get_task().get_id())
                # print("--------------------------")
                synergy_tot = 0.0
                for parallel_agent_task_sol in problem_solution_agent["human_right_arm"]:
                    # print(parallel_agent_task_sol.get_task().get_id())
                    parallel_agent_task_sol: TaskSolution

                    overlapping = min(parallel_agent_task_sol.get_end_time(), main_agent_task_sol.get_end_time()) - max(parallel_agent_task_sol.get_start_time(), main_agent_task_sol.get_start_time())
                    if overlapping <= 0:
                        overlapping = 0
                    # print(overlapping)
                    synergy = main_agent_task_sol.get_task().get_synergy("ur5_on_guide","human_right_arm",parallel_agent_task_sol.get_task().get_type())
                    synergy_index = overlapping*synergy
                    # print(f"Synergy index : {synergy_index}")
                    synergy_tot += synergy_index
            parallel_agent = "human_right_arm"
            main_agent = "ur5_on_guide"
            print(f"Synergy tot: {synergy_tot}")
