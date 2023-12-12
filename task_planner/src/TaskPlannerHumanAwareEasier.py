# from Problem import Problem
# from Task import TaskSolution
# from TaskPlannerHumanAware import TaskPlannerHumanAware
#
# from dataclasses import dataclass, field
# from typing import List, Dict, Optional
# from utils import Behaviour, Objective
#
# import gurobipy as gp
# import itertools
#
# EPS = 1E-12
# BIG_M = 1E12
#
#
# @dataclass
# class TaskPlannerHumanAwareEasier(TaskPlannerHumanAware):
#     # def add_t_end_constraints(self, agent_task_combination, cost) -> None:
#     #     super(TaskPlannerHumanAware, self).add_t_end_constraints(agent_task_combination, cost)
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
#             self.model.addConstr(self.decision_variables["t_end"][task] == t_end[task], name=f"t_end({task})")
#
#     def set_objective(self) -> None:
#         cost_function = self.model.addVar(name="J", vtype=gp.GRB.CONTINUOUS)
#         upper_bound = self.problem_definition.get_nominal_upper_bound()
#         t_end_robot = self.model.addVar(name="t_end_robot", vtype=gp.GRB.CONTINUOUS, lb=0, ub=upper_bound)
#         t_end_human = self.model.addVar(name="t_end_human", vtype=gp.GRB.CONTINUOUS, lb=0, ub=upper_bound)
#         t_start_robot = self.model.addVar(name="t_start_robot", vtype=gp.GRB.CONTINUOUS, lb=0, ub=upper_bound)
#         t_start_human = self.model.addVar(name="t_start_human", vtype=gp.GRB.CONTINUOUS, lb=0, ub=upper_bound)
#         idle_human = self.model.addVar(name="idle_human", vtype=gp.GRB.CONTINUOUS, lb=0, ub=upper_bound)
#         idle_robot = self.model.addVar(name="idle_robot", vtype=gp.GRB.CONTINUOUS, lb=0, ub=upper_bound)
#
#         prova = [
#             self.decision_variables["assignment"][(agent, task)] * self.decision_variables["t_end"][task] for
#             (agent, task) in self.decision_variables["assignment"].keys() if agent == "ur5_on_guide"]
#         prova_start = [
#             self.decision_variables["assignment"][(agent, task)] * self.decision_variables["t_start"][task] for
#             (agent, task) in self.decision_variables["assignment"].keys() if agent == "ur5_on_guide"]
#
#         aux = []
#         aux_start = []
#         for idx, k in enumerate(prova):
#             aux.append(self.model.addVar(name=str(k), vtype=gp.GRB.CONTINUOUS, lb=0, ub=upper_bound))
#             self.model.addConstr(aux[idx] == k)
#             aux_start.append(self.model.addVar(name="start"+str(k), vtype=gp.GRB.CONTINUOUS, lb=0, ub=upper_bound))
#             self.model.addConstr(aux_start[idx] == prova_start[idx])
#
#         self.model.addConstr(t_end_robot == (gp.max_(aux)))
#         self.model.addConstr(t_start_robot == (gp.max_(aux_start)))
#
#         prova_hum = [
#             self.decision_variables["assignment"][(agent, task)] * self.decision_variables["t_end"][task] for
#             (agent, task) in self.decision_variables["assignment"].keys() if agent == "human_right_arm"]
#         prova_hum_start = [
#             self.decision_variables["assignment"][(agent, task)] * self.decision_variables["t_start"][task] for
#             (agent, task) in self.decision_variables["assignment"].keys() if agent == "human_right_arm"]
#
#         aux_hum = []
#         aux_start_hum = []
#         for idx, k in enumerate(prova_hum):
#             aux_hum.append(self.model.addVar(name=str(k), vtype=gp.GRB.CONTINUOUS, lb=0, ub=upper_bound))
#             self.model.addConstr(aux_hum[idx] == k)
#             aux_start_hum.append(self.model.addVar(name="start"+str(k), vtype=gp.GRB.CONTINUOUS, lb=0, ub=upper_bound))
#             self.model.addConstr(aux_start_hum[idx] == prova_hum_start[idx])
#
#         self.model.addConstr(t_end_human == (gp.max_(aux_hum)))
#         self.model.addConstr(t_start_human == (gp.max_(aux_start_hum)))
#         aux_somma_parziale_human = self.model.addVar(name="aux_somma_parziale_human",lb=0,ub=upper_bound)
#         aux_somma_parziale_robot = self.model.addVar(name="aux_somma_parziale_robot", lb=0, ub=upper_bound)
#
#         self.model.addConstr(aux_somma_parziale_human == gp.quicksum(prova_hum) - gp.quicksum(prova_hum_start))
#         self.model.addConstr(aux_somma_parziale_robot == gp.quicksum(prova) - gp.quicksum(prova_start))
#
#         self.model.addConstr(idle_robot >= -0.1 + t_end_robot + aux_somma_parziale_robot)
#         self.model.addConstr(idle_robot <= 0.1 + t_end_robot + aux_somma_parziale_robot)
#
#         # self.model.addConstr(idle_human >= -0.1 + t_end_human + aux_somma_parziale_human)
#         # self.model.addConstr(idle_human <= 0.1 + t_end_human + aux_somma_parziale_human)
#
#         self.model.update()
#         sigma_index = self.model.addVar(name="sigma_index", vtype=gp.GRB.CONTINUOUS)
#
#         parallel_agent = "human_right_arm"
#         main_agent = "ur5_on_guide"
#         # TODO: Set two agents in "init"
#         tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
#         tasks_synergies = self.problem_definition.get_tasks_synergies()
#
#         sigma_val = 0.0
#         sigma_tot = 0.0
#         _, cost = gp.multidict(self.problem_definition.get_combinations())
#         for task in self.decision_variables["t_start"].keys():
#             for task_pair in self.decision_variables["overlapping"].keys():
#                 if task in task_pair:
#                     parallel_task = set(task_pair).difference({task}).pop()
#                     task_type = tasks_type_correspondence[task]
#                     parallel_task_type = tasks_type_correspondence[parallel_task]
#                     overlapping = self.decision_variables["overlapping"][task_pair]
#                     if (main_agent, task) in self.decision_variables["assignment"].keys() and \
#                             (parallel_agent, parallel_task) in self.decision_variables["assignment"].keys():
#                         assignment_main = self.decision_variables["assignment"][(main_agent, task)]
#                         # assignment_parallel = self.decision_variables["assignment"][(parallel_agent, parallel_task)]
#                         if (task_type, main_agent, parallel_task_type, parallel_agent) in tasks_synergies:
#                             synergy = tasks_synergies[(task_type, main_agent, parallel_task_type, parallel_agent)]
#                             # if (main_agent, parallel_task) in cost.keys(): Se discreto ci vuole il cost
#                             if self.behaviour == Behaviour.CONTINUOUS:
#                                 sigma_val += overlapping * (synergy - 1) * assignment_main
#                             elif self.behaviour == Behaviour.DISCRETE:
#                                 sigma_val += overlapping * (synergy - 1) * assignment_main * cost[(main_agent, task)]
#                             sigma_tot += synergy
#         self.model.addConstr(sigma_index == sigma_val / 1)
#
#         cost_function = self.model.addVar(name="cost_function", vtype=gp.GRB.CONTINUOUS)
#         cost_function_temp = self.model.addVar(name="cost_function_temp", vtype=gp.GRB.CONTINUOUS)
#
#         if self.objective == Objective.SUM_T_START_END:
#             self.model.addConstr(cost_function_temp == gp.quicksum(self.decision_variables["t_end"]) +
#                                  gp.quicksum(self.decision_variables["t_start"]))
#             # self.model.addConstr(cost_function == cost_function_temp + sigma_index )
#         elif self.objective == Objective.MAKESPAN:
#             self.model.addGenConstrMax(cost_function_temp, self.decision_variables["t_end"])
#             # self.model.addConstr(cost_function_temp == gp.max_(self.decision_variables["t_end"]) )
#             self.model.addConstr(cost_function == cost_function_temp)
#         elif self.objective == Objective.SUM_T_START:
#             self.model.addConstr(cost_function_temp == gp.quicksum(self.decision_variables["t_start"]))
#             self.model.addConstr(cost_function == cost_function_temp + sigma_index)
#         elif self.objective == Objective.SUM_T_END:
#             self.model.addConstr(cost_function_temp == gp.quicksum(self.decision_variables["t_end"]))
#             self.model.addConstr(cost_function == cost_function_temp + sigma_index)
#         elif self.objective == Objective.SYNERGY:
#             self.model.addConstr(cost_function_temp == gp.quicksum(self.decision_variables["t_end"]))
#             self.model.addConstr(cost_function == sigma_index )
#
#         self.model.setObjective(cost_function, gp.GRB.MINIMIZE)
#         # self.model.setObjectiveN(sigma_index,gp.GRB.MINIMIZE,2)
#         # self.model.setObjectiveN(cost_function,gp.GRB.MINIMIZE,1)
from Problem import Problem
from Task import TaskSolution
from TaskPlannerHumanAware import TaskPlannerHumanAware

from dataclasses import dataclass, field
from typing import List, Dict, Optional
from utils import Behaviour, Objective

import gurobipy as gp
import itertools

EPS = 1E-4
BIG_M = 1E4


@dataclass
class TaskPlannerHumanAwareEasier(TaskPlannerHumanAware):
    def add_t_end_constraints(self, agent_task_combination, cost) -> None:
        super(TaskPlannerHumanAware, self).add_t_end_constraints(agent_task_combination, cost)

    def set_objective(self) -> None:
        cost_function = self.model.addVar(name="J", vtype=gp.GRB.CONTINUOUS)

        sigma_index = self.model.addVar(name="sigma_index", vtype=gp.GRB.CONTINUOUS, lb=-50) # -2)

        robot_agents = self.problem_definition.get_robot_agents()
        human_agents = self.problem_definition.get_not_robot_agents()

        tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
        tasks_synergies = self.problem_definition.get_tasks_synergies()

        sigma_val = 0.0
        sigma_tot = 0.0
        _, cost = gp.multidict(self.problem_definition.get_combinations())
        
        for robot_agent in robot_agents:
            for human_agent in human_agents:
                main_agent = robot_agent
                parallel_agent = human_agent

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
        self.cost_to_print = cost_function_temp

        if self.objective == Objective.SUM_T_START_END:
            self.model.addConstr(cost_function_temp == gp.quicksum(self.decision_variables["t_end"]) +
                                 gp.quicksum(self.decision_variables["t_start"]))
            self.model.addConstr(cost_function == cost_function_temp + sigma_index)
        elif self.objective == Objective.MAKESPAN:
            self.model.addConstr(cost_function_temp == gp.max_(self.decision_variables["t_end"]))
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
