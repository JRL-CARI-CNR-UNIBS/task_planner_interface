from Problem import Problem
from Task import TaskSolution
from TaskPlanner import TaskPlanner
from TaskPlannerHumanAware import TaskPlannerHumanAware

from dataclasses import dataclass, field
from typing import List, Dict, Optional

import gurobipy as gp
import itertools

from utils import Behaviour, Objective

EPS = 1E-12
BIG_M = 1E12


@dataclass
class TaskPlannerSynergisticEasier(TaskPlannerHumanAware):
    behaviour: Behaviour = field(default=Behaviour.CONTINUOUS)

    # objective: Objective

    def add_t_end_constraints(self, agent_task_combination, cost) -> None:
        super(TaskPlannerHumanAware, self).add_t_end_constraints(agent_task_combination, cost)

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
                (check_parallelism == 1) >> (2 * ov_ik == d_i + d_k - abs_sum_delta_ik))
            self.model.addConstr((check_parallelism == 0) >> (ov_ik == 0))

    def set_objective(self) -> None:
        # TODO: Can be put in TaskPlanner Base class.
        cost_function = self.model.addVar(name="J", vtype=gp.GRB.CONTINUOUS)
        sigma_index = self.model.addVar(name="sigma_index", vtype=gp.GRB.CONTINUOUS, lb=0)
        print(sigma_index)
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
                                sigma_val += overlapping * (synergy - 1) * assignment_main * cost[
                                    (main_agent, task)]
                            sigma_tot += synergy
        self.model.addConstr(sigma_index == sigma_val / 1)

        cost_function = self.model.addVar(name="cost_function", vtype=gp.GRB.CONTINUOUS)
        cost_function_temp = self.model.addVar(name="cost_function_temp", vtype=gp.GRB.CONTINUOUS)

        if self.objective == Objective.SUM_T_START_END:
            self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["t_end"]) +
                                 gp.quicksum(self.decision_variables["t_start"]))
        elif self.objective == Objective.MAKESPAN:
            self.model.addConstr(cost_function_temp == gp.max_(self.decision_variables["t_end"]))
            self.model.addConstr(cost_function == cost_function_temp + sigma_index)
            # self.model.addGenConstrMax(cost_function, self.decision_variables["t_end"])
        elif self.objective == Objective.SUM_T_START:
            self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["t_start"]))
        elif self.objective == Objective.SUM_T_END:
            self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["t_end"]))
        elif self.objective == Objective.SYNERGY:
            pass
        #     upper_bound = self.problem_definition.get_nominal_upper_bound()
        #     parallel_agent = "human_right_arm"
        #     main_agent = "ur5_on_guide"
        #     t_start_robot = self.model.addVars(list(self.decision_variables["t_start"].keys()), lb=0, ub=upper_bound,
        #                                        name="t_start_robot", vtype=gp.GRB.CONTINUOUS)
        #     t_start_human = self.model.addVars(list(self.decision_variables["t_start"].keys()), lb=0, ub=upper_bound,
        #                                        name="t_start_human", vtype=gp.GRB.CONTINUOUS)
        #     t_end_robot = self.model.addVars(list(self.decision_variables["t_end"].keys()), lb=0, ub=upper_bound,
        #                                      name="t_end_robot", vtype=gp.GRB.CONTINUOUS)
        #     t_end_human = self.model.addVars(list(self.decision_variables["t_end"].keys()), lb=0, ub=upper_bound,
        #                                      name="t_end_human", vtype=gp.GRB.CONTINUOUS)
        #     duration_human = self.model.addVar(lb=0, ub=upper_bound * 10, name="duration_human",
        #                                        vtype=gp.GRB.CONTINUOUS)
        #     duration_robot = self.model.addVar(lb=0, ub=upper_bound * 10, name="duration_human",
        #                                        vtype=gp.GRB.CONTINUOUS)
        #
        #     for task in self.decision_variables["t_start"].keys():
        #         if (main_agent, task) in self.decision_variables["assignment"].keys():
        #             self.model.addConstr(
        #                 t_start_robot[task] == self.decision_variables["assignment"][(main_agent, task)] *
        #                 self.decision_variables["t_start"][task])
        #             self.model.addConstr(
        #                 t_end_robot[task] == self.decision_variables["assignment"][(main_agent, task)] *
        #                 self.decision_variables["t_end"][task])
        #         else:
        #             self.model.addConstr(t_start_robot[task] == 0)
        #             self.model.addConstr(t_end_robot[task] == 0)
        #         if (parallel_agent, task) in self.decision_variables["assignment"].keys():
        #             self.model.addConstr(
        #                 t_start_human[task] == self.decision_variables["assignment"][(parallel_agent, task)] *
        #                 self.decision_variables["t_start"][task])
        #             self.model.addConstr(
        #                 t_end_human[task] == self.decision_variables["assignment"][(parallel_agent, task)] *
        #                 self.decision_variables["t_end"][task])
        #         else:
        #             self.model.addConstr(t_start_human[task] == 0)
        #             self.model.addConstr(t_end_human[task] == 0)
        #
        #     self.model.addConstr(duration_robot == gp.max_(t_end_robot))
        #     self.model.addConstr(duration_human == gp.max_(t_end_human))
        #
        #     tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
        #     tasks_synergies = self.problem_definition.get_tasks_synergies()
        #
        #     sigma_val = 0.0
        #     sigma_tot = 0.0
        #     sigma_var = self.model.addVar(name="sigma_var", vtype=gp.GRB.CONTINUOUS, lb=-100)
        #     _, cost = gp.multidict(self.problem_definition.get_combinations())
        #     for task in self.decision_variables["t_start"].keys():
        #         for task_pair in self.decision_variables["overlapping"].keys():
        #             if task in task_pair:
        #                 parallel_task = set(task_pair).difference({task}).pop()
        #                 task_type = tasks_type_correspondence[task]
        #                 parallel_task_type = tasks_type_correspondence[parallel_task]
        #                 overlapping = self.decision_variables["overlapping"][task_pair]
        #                 if (main_agent, task) in self.decision_variables["assignment"].keys() and \
        #                         (parallel_agent, parallel_task) in self.decision_variables["assignment"].keys():
        #                     assignment_main = self.decision_variables["assignment"][(main_agent, task)]
        #                     if (task_type, main_agent, parallel_task_type, parallel_agent) in tasks_synergies:
        #                         synergy = tasks_synergies[(task_type, main_agent, parallel_task_type, parallel_agent)]
        #                         if self.behaviour == Behaviour.CONTINUOUS:
        #                             sigma_val += overlapping * (synergy - 1) * assignment_main
        #                         elif self.behaviour == Behaviour.DISCRETE:
        #                             sigma_val += overlapping * (synergy - 1) * assignment_main * cost[
        #                                 (main_agent, task)]
        #                         # sigma_tot += synergy
        #     # duration = self.model.addVar(lb=0,
        #     #                              ub=self.problem_definition.get_nominal_upper_bound()*5,
        #     #                              name="duration",
        #     #                              vtype=gp.GRB.CONTINUOUS)
        #     idle_human = self.model.addVar(name="idle_human", lb=0,
        #                                    ub=self.problem_definition.get_nominal_upper_bound() * 5,
        #                                    vtype=gp.GRB.CONTINUOUS)
        #     idle_robot = self.model.addVar(name="idle_robot", lb=0,
        #                                    ub=self.problem_definition.get_nominal_upper_bound() * 5,
        #                                    vtype=gp.GRB.CONTINUOUS)
        #
        #     # self.model.addConstr(duration == gp.max_(self.decision_variables["t_end"]))
        #     # self.model.addConstr(cost_function >= (-3 +
        #     #                                        sigma_val + duration + duration - gp.quicksum(
        #     #             self.decision_variables["t_end"]) + gp.quicksum(
        #     #             self.decision_variables["t_start"])))
        #     # self.model.addConstr(cost_function <= (3 +
        #     #                                        sigma_val + duration + duration - gp.quicksum(
        #     #             self.decision_variables["t_end"]) + gp.quicksum(
        #     #             self.decision_variables["t_start"])))
        #     # self.model.addConstr(cost_function == (duration_robot + duration_human +
        #     #                                        sigma_val - gp.quicksum(t_end_robot) - gp.quicksum(t_end_human)
        #     #                                        + gp.quicksum(t_start_robot) + gp.quicksum(t_start_human)))
        #     self.model.addConstr(idle_robot == duration_robot - gp.quicksum(t_end_robot) + gp.quicksum(t_start_robot))
        #     self.model.addConstr(idle_human == duration_human - gp.quicksum(t_end_human) + gp.quicksum(t_start_human))
        #     self.model.addConstr(sigma_var == sigma_val)
        #     self.model.addConstr(cost_function == sigma_val + idle_human + idle_robot)
        #
        #     # self.model.addConstr(cost_function == sigma_val )
        #
        #     # print(self.model.getVars())
        #
        #     # self.model.addConstr(duration == gp.max_(self.decision_variables["t_end"]))
        #
        #     # self.model.addConstr(duration <= 95)
        #
        # # Objective function
        # self.model.setObjective(cost_function, gp.GRB.MINIMIZE)
        # #
        # # self.model.write("Model.lp")
        # # self.model.tune()
