from Problem import Problem
from Task import TaskSolution
from TaskPlanner import TaskPlanner

from dataclasses import dataclass, field
from typing import List, Dict, Optional

import gurobipy as gp
import itertools

from utils import Behaviour, Objective

EPS = 1E-12
BIG_M = 1E12


@dataclass
class TaskPlannerSynergisticBand(TaskPlanner):
    behaviour: Behaviour = field(default=Behaviour.CONTINUOUS)
    epsilon: float = field(default=0.0)
    relaxed: bool = field(default=False)

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
        self.decision_variables["d_i_h_tilde"] = self.model.addVars(tasks_list,
                                                                    name="d_i_h_tilde",
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
        super().create_model()
        self.add_overlapping_constraints()

    def add_t_end_constraints(self, agent_task_combination, cost) -> None:
        agent_h = "human_right_arm"
        agent_r = "ur5_on_guide"
        tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
        tasks_synergies = self.problem_definition.get_tasks_synergies()

        d_i_r_tilde = {}
        for task_i in self.decision_variables["t_start"].keys():

            if (agent_h, task_i) not in self.decision_variables["assignment"].keys():
                duration_i_h = 0
                # self.model.addConstr(self.decision_variables["d_i_h_tilde"][task_i] == 0)
            else:
                a_i_h = self.decision_variables["assignment"][(agent_h, task_i)]
                d_i_h_hat = cost[(agent_h, task_i)]
                duration_i_h = a_i_h * d_i_h_hat
                # self.model.addConstr((a_i_h == 1) >> (self.decision_variables["d_i_h_tilde"][task_i] >= duration_i_h*(1-self.eps)))
                # self.model.addConstr((a_i_h == 1) >> (self.decision_variables["d_i_h_tilde"][task_i] <= duration_i_h*(1+self.eps)))
                # self.model.addConstr((a_i_h == 0) >> (self.decision_variables["d_i_h_tilde"][task_i] == 0))
            self.model.addConstr(self.decision_variables["d_i_h_tilde"][task_i] >= duration_i_h * (1 - self.epsilon))
            self.model.addConstr(self.decision_variables["d_i_h_tilde"][task_i] <= duration_i_h * (1 + self.epsilon))
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
                if self.relaxed:
                    self.model.addConstr(self.decision_variables["d_i_r_tilde"][task_i] >= d_i_r_tilde[task_i])
                else:
                    self.model.addConstr(self.decision_variables["d_i_r_tilde"][task_i] == d_i_r_tilde[task_i])
                self.model.addConstr(self.decision_variables["d_i_r_tilde"][task_i] >= 0.7 * cost[(agent_r, task_i)])

                # self.model.addConstr(self.decision_variables["d_i_h_tilde"][task_i] >= duration_i_h * (1 - self.epsilon))
                # self.model.addConstr(self.decision_variables["d_i_h_tilde"][task_i] <= duration_i_h * (1 + self.epsilon))

                duration_i_r = self.decision_variables["d_i_r_tilde"][task_i] * self.decision_variables["assignment"][
                    (agent_r, task_i)]
            d_i = self.decision_variables["duration"][task_i]

            self.model.addConstr(d_i == self.decision_variables["d_i_h_tilde"][task_i] + duration_i_r)
            self.model.addConstr(
                d_i == self.decision_variables["t_end"][task_i] - self.decision_variables["t_start"][task_i])

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

    def get_solution(self) -> List[TaskSolution]:
        for v in self.model.getVars():
            if "idle" in v.varName or "min_tend" in v.varName or "duration" in v.varName or "tStart" in v.varName or "assignment" in v.varName or "tot" in v.varName or "t_end_robot" in v.varName or "t_end_human" in v.varName or "t_end" in v.varName or "t_start" in v.varName or "overlapping" in v.varName or "d_i_r_tilde" in v.varName or "d_i_h_tilde" in v.varName:
                print(v.varName, v.x)

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
                    if len(decision_variable) == 1 and decision_variable[0].X == 1:
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

    def set_objective(self) -> None:
        # TODO: Can be put in TaskPlanner Base class.
        cost_function = self.model.addVar(name="J", vtype=gp.GRB.CONTINUOUS)

        if self.objective == Objective.SUM_T_START_END:
            self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["t_end"]) +
                                 gp.quicksum(self.decision_variables["t_start"]))
        elif self.objective == Objective.MAKESPAN:
            self.model.addGenConstrMax(cost_function, self.decision_variables["t_end"])
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
            self.model.addConstr(cost_function == sigma_val )

            # self.model.addConstr(duration == gp.max_(self.decision_variables["t_end"]))

            # self.model.addConstr(duration <= 95)

        else:
            self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["d_i_r_tilde"]))

        # Objective function
        self.model.setObjective(cost_function, gp.GRB.MINIMIZE)

        self.model.write("Model.lp")
        # self.model.tune()
