from Problem import Problem
from Task import TaskSolution
from TaskPlanner import TaskPlanner

from dataclasses import dataclass, field
from typing import List, Dict, Optional

import gurobipy as gp
import itertools

from utils import Behaviour, Objective

EPS = 1E-3
BIG_M = 1E3


@dataclass
class TaskPlannerHumanAwareRelaxed(TaskPlanner):
    behaviour: Behaviour = field(default=Behaviour.CONTINUOUS)

    def create_model(self) -> None:
        tasks_list = self.problem_definition.get_tasks_list()
        tasks_pairs = itertools.combinations(tasks_list, 2)
        upper_bound = self.problem_definition.get_nominal_upper_bound()

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
                                                                        ub=15)
        super().create_model()
        self.add_overlapping_constraints()
        # self.model.setParam("cutoff",105)
        # self.model.setParam("branchdir",-1)
        # self.model.setParam("disconnected",2)
        # self.model.setParam("NLPHeur", 1)
        # self.model.setParam("Cuts", 3)
        # self.model.setParam("MIPFocus", 3)

        # self.model.setParam("heuristics", 0.001)
        # self.model.setParam("MIPGapAbs", 0.28)

    # def add_t_end_constraints(self, agent_task_combination, cost) -> None:
    #     t_end = {}
    #     for agent, task in agent_task_combination:
    #         if task not in t_end:
    #             t_end[task] = self.decision_variables["t_start"][task]
    #         t_end[task] += self.decision_variables["assignment"][(agent, task)] * cost[(agent, task)]
    #     tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
    #     tasks_synergies = self.problem_definition.get_tasks_synergies()
    #
    #     parallel_agent = "human_right_arm"
    #     main_agent = "ur5_on_guide"
    #     # TODO: define agents in "init"
    #     for task in self.decision_variables["t_start"].keys():
    #
    #         for task_pair in self.decision_variables["overlapping"].keys():
    #             if task in task_pair:
    #                 parallel_task = set(task_pair).difference({task}).pop()
    #                 task_type = tasks_type_correspondence[task]
    #                 parallel_task_type = tasks_type_correspondence[parallel_task]
    #                 overlapping = self.decision_variables["overlapping"][task_pair]
    #                 if (main_agent, task) in self.decision_variables["assignment"].keys():
    #                     assignment_main_task = self.decision_variables["assignment"][(main_agent, task)]
    #                     if (task_type, main_agent, parallel_task_type, parallel_agent) in tasks_synergies:
    #                         synergy = tasks_synergies[(task_type, main_agent, parallel_task_type, parallel_agent)]
    #                         if self.behaviour == Behaviour.CONTINUOUS:
    #                             t_end[task] += overlapping * (synergy - 1) * assignment_main_task
    #                         elif self.behaviour == Behaviour.DISCRETE:  # Add cost (Measurement: time)
    #                             t_end[task] += overlapping * (synergy - 1) * assignment_main_task * cost[
    #                                 (main_agent, task)]
    #         self.model.addConstr(self.decision_variables["t_end"][task] == t_end[task], name=f"t_end({task})")
    #
    #         # TODO: Pensare a cosa simile a quella sotto: aggiungere vincolo = t_end MODIFICATO SOLO se parallel task è assegnato alla persona.
    #         # Note: Così come commentato non è corretto perchè nel caso non lo fosse dovrebbe essere uguale a t_start + d*a.
    #         # Bisognerebbe fare vincolo sopra e poi aggiungerlo sotto, ma si riferirà allo stesso vincolo o agigunge un'altro?
    #         # self.model.addConstr(self.decision_variables["overlapping"][parallel_task] == 1 >>
    #         #                      (self.decision_variables["t_end"][task] == t_end[task]),
    #         #                      name=f"t_end({task})")

    def add_overlapping_constraints(self, upper_bound=None):

        for task_i, task_j in self.decision_variables["overlapping"].keys():
            t_end_i = self.decision_variables["t_end"][task_i]
            t_end_j = self.decision_variables["t_end"][task_j]
            t_start_i = self.decision_variables["t_start"][task_i]
            t_start_j = self.decision_variables["t_start"][task_j]
            upper_bound = self.problem_definition.get_nominal_upper_bound()
            # TODO: Remove from here upper bound
            raw_overlapping = self.model.addVar(name=f"raw_overlapping({task_i},{task_j})",
                                                vtype=gp.GRB.CONTINUOUS,
                                                lb=-upper_bound,
                                                ub=upper_bound)
            min_t_end = self.model.addVar(name=f"min_t_end({task_i}, {task_j})",
                                          vtype=gp.GRB.CONTINUOUS,
                                          lb=0,
                                          ub=upper_bound)
            max_t_start = self.model.addVar(name=f"max_t_start({task_i}, {task_j})",
                                            vtype=gp.GRB.CONTINUOUS,
                                            lb=0,
                                            ub=upper_bound)

            self.model.addGenConstrMin(min_t_end, [t_end_i, t_end_j])
            self.model.addGenConstrMax(max_t_start, [t_start_i, t_start_j])

            self.model.addConstr(raw_overlapping == (min_t_end - max_t_start))

            check_parallelism = self.model.addVar(name=f"check_parallelism({task_i},{task_j})",
                                                  vtype=gp.GRB.BINARY, lb=0, ub=1)

            self.model.addConstr(raw_overlapping >= -BIG_M * (1 - check_parallelism))
            self.model.addConstr(raw_overlapping <= BIG_M * check_parallelism - EPS)

            if self.behaviour == Behaviour.DISCRETE:
                self.model.addConstr((self.decision_variables["overlapping"][(task_i, task_j)] == check_parallelism))
            elif self.behaviour == Behaviour.CONTINUOUS:
                self.model.addConstr((self.decision_variables["overlapping"][(task_i, task_j)] == (
                        min_t_end - max_t_start) * check_parallelism))

    def set_objective(self) -> None:
        # TODO: Can be put in TaskPlanner Base class.
        cost_function = self.model.addVar(name="J", vtype=gp.GRB.CONTINUOUS)

        if self.objective == Objective.SUM_T_START_END:
            self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["t_end"]) +
                                 gp.quicksum(self.decision_variables["t_start"]))
        elif self.objective == Objective.MAKESPAN:
            # self.model.addGenConstrMax(cost_function, self.decision_variables["t_end"])
            parallel_agent = "human_right_arm"
            main_agent = "ur5_on_guide"

            upper_bound = self.problem_definition.get_nominal_upper_bound()
            tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
            tasks_synergies = self.problem_definition.get_tasks_synergies()

            sigma_val = 0.0
            _, cost = gp.multidict(self.problem_definition.get_combinations())
            sinergy_index = self.model.addVar(name=f"Sinergy_Index",
                                              vtype=gp.GRB.CONTINUOUS,
                                              lb=-10,
                                              ub=20)
            self.synergy_index_to_print = sinergy_index
            makespan = self.model.addVar(name=f"makespan",
                                              vtype=gp.GRB.CONTINUOUS,
                                              lb=0,
                                              ub=upper_bound * 5)
            self.makespan_to_print = makespan

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
                                S_ij = self.model.addVar(name=f"S_ij({task_pair})",
                                                         vtype=gp.GRB.CONTINUOUS,
                                                         lb=-5,
                                                         ub=15)
                                synergy = tasks_synergies[(task_type, main_agent, parallel_task_type, parallel_agent)]
                                self.model.addConstr((assignment_main == 1) >> (S_ij == overlapping * (synergy - 1)))
                                self.model.addConstr((assignment_main == 0) >> (S_ij == 0))
                                # self.model.addConstr((assignment_main == 1) >> (S_ij + BIG_M * (1-assignment_main) >= overlapping * (synergy - 1)))
                                # self.model.addConstr(S_ij + BIG_M * (1-assignment_main) >= overlapping * (synergy - 1))

                                sigma_val += S_ij
                                # self.model.addConstr(
                                #     S_ij + BIG_M * (1 - assignment_main) >= overlapping * (synergy - 1))
            self.model.addConstr(sinergy_index == sigma_val)
            self.model.addConstr(makespan == gp.max_(self.decision_variables["t_end"]))
            self.model.addConstr(cost_function == makespan + sinergy_index)


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
                if (main_agent, task) in self.decision_variables["assignment"].keys():
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
                    self.model.addConstr(
                        t_start_human[task] == self.decision_variables["assignment"][(parallel_agent, task)] *
                        self.decision_variables["t_start"][task])
                    self.model.addConstr(
                        t_end_human[task] == self.decision_variables["assignment"][(parallel_agent, task)] *
                        self.decision_variables["t_end"][task])
                else:
                    self.model.addConstr(t_start_human[task] == 0)
                    self.model.addConstr(t_end_human[task] == 0)

            self.model.addConstr(duration_robot == gp.max_(t_end_robot))
            self.model.addConstr(duration_human == gp.max_(t_end_human))

            tasks_type_correspondence = self.problem_definition.get_tasks_type_correspondence()
            tasks_synergies = self.problem_definition.get_tasks_synergies()

            sigma_val = 0.0
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

            self.model.addConstr(idle_robot == duration_robot - gp.quicksum(t_end_robot) + gp.quicksum(t_start_robot))
            self.model.addConstr(idle_human == duration_human - gp.quicksum(t_end_human) + gp.quicksum(t_start_human))
            self.model.addConstr(cost_function == sigma_val + idle_human + idle_robot)

            # self.model.addConstr(duration == gp.max_(self.decision_variables["t_end"]))

            # self.model.addConstr(duration <= 95)

        # Objective function
        self.model.setObjective(cost_function, gp.GRB.MINIMIZE)


        self.model.write("Model.lp")
        # self.model.tune()

    def callback(self, model, where):
        # print(where)
        # print(model)
        if where == gp.GRB.Callback.MIPSOL:
            print(f"Sigma index: {model.cbGetSolution(self.synergy_index_to_print)}")
            print(f"Makespan function: {model.cbGetSolution(self.makespan_to_print)}")

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
                    if (agent,task) in self.decision_variables["assignment"].keys():
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
            # show_timeline(problem_solution)
            synergy_val = self.compute_synergy_val(problem_solution)
            print(f"Total synergy of solution: {synergy_val}")