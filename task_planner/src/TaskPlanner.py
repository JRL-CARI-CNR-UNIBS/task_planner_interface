from Problem import Problem
from Task import TaskSolution

from dataclasses import dataclass, field
from typing import List, Dict, Optional

import gurobipy as gp
import itertools

EPS = 1E-6
BIG_M = 1E7


@dataclass
class TaskPlanner:
    name: str
    problem_definition: Problem
    model: gp.Model = field(init=False)
    decision_variables: Dict[str, gp.tupledict] = field(init=False)

    use_synergy: bool = False

    # alpha: float = 0.0

    def __post_init__(self):
        if self.problem_definition.consistency_check() is not True:
            raise ValueError("The problem is not consistent. Check it!")

    def initialize(self):
        e = gp.Env(empty=True)
        if False:
            e.setParam('WLSACCESSID', wls_access_id)
            e.setParam('WLSSECRET', wls_secret)
            e.setParam('LICENSEID', license_id)
        e.start()

        # Create the model within the Gurobi environment
        self.model = gp.Model(self.name, env=e)
        self.decision_variables = {}

    def create_model(self) -> None:
        agent_task_combination, cost = gp.multidict(self.problem_definition.get_combinations())
        tasks_list = self.problem_definition.get_tasks_list()

        upper_bound = self.problem_definition.get_nominal_upper_bound() * 2

        # TODO: Ragionare sulle possibili combination
        tasks_pairs = itertools.combinations(tasks_list, 2)
        tasks_per_agent = self.problem_definition.get_tasks_per_agent()

        couple_of_tasks_per_agent = \
            [list(itertools.product(itertools.combinations(tasks_per_agent[agent], 2), [agent])) for agent in
             tasks_per_agent.keys()]
        couple_of_tasks_per_agent = [item for sublist in couple_of_tasks_per_agent for item in sublist]
        # print(couple_of_tasks_per_agent)

        # Decision variables
        self.decision_variables["assignment"] = self.model.addVars(agent_task_combination,
                                                                   name="assignment",
                                                                   vtype=gp.GRB.BINARY)
        # print(tasks_list)
        self.decision_variables["t_start"] = self.model.addVars(tasks_list,
                                                                name="t_start",
                                                                vtype=gp.GRB.CONTINUOUS,
                                                                lb=0, ub=upper_bound)
        self.decision_variables["t_end"] = self.model.addVars(tasks_list,
                                                              name="t_end",
                                                              vtype=gp.GRB.CONTINUOUS,
                                                              lb=0, ub=upper_bound)

        self.decision_variables["delta_ij"] = self.model.addVars(couple_of_tasks_per_agent,
                                                                 name="same_agent",
                                                                 vtype=gp.GRB.BINARY)

        self.add_t_end_constraints(agent_task_combination, cost)
        self.add_assignment_constraints(tasks_list)

        # if self.use_synergy:
        #     self.decision_variables["overlapping"] = self.model.addVars(tasks_pairs,
        #                                                                 name="overlapping",
        #                                                                 vtype=gp.GRB.CONTINUOUS,
        #                                                                 lb=-gp.GRB.INFINITY,
        #                                                                 ub=gp.GRB.INFINITY)

    def add_assignment_constraints(self, tasks_list):
        # Unique task-agent assignment
        self.model.addConstrs(
            (self.decision_variables["assignment"].sum('*', task) == 1 for task in tasks_list),
            name='unique_assignment')

        # Assignments constraints
        for task, not_enabled_agents in self.problem_definition.get_not_enabled_agents_constraints().items():
            print(task, not_enabled_agents)
            for agent in not_enabled_agents:
                self.model.addConstr(self.decision_variables["assignment"][(agent, task)] == 0,
                                     name=f'not_enabled_assignment_{task}')

    def add_t_end_constraints(self, agent_task_combination, cost) -> None:
        # Tend constraints
        t_end = {}
        print(agent_task_combination)
        for agent, task in agent_task_combination:
            if task not in t_end:
                t_end[task] = self.decision_variables["t_start"][task]
            t_end[task] += self.decision_variables["assignment"][(agent, task)] * cost[(agent, task)]

        [self.model.addConstr(
            self.decision_variables["t_end"][task] == t_end_value) for task, t_end_value in t_end.items()]

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

    def set_objective(self) -> None:
        # Makespan
        makespan = self.model.addVar(name="makespan", vtype=gp.GRB.CONTINUOUS)

        self.model.addConstr(makespan == gp.max_(self.decision_variables["t_end"]))

        # Objective function
        self.model.setObjective(makespan, gp.GRB.MINIMIZE)
        self.model.write('MODEL.lp')

    def solve(self) -> None:
        # Optimization
        self.model.params.NonConvex = 2
        self.model.optimize()

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
            try:
                task_solution = self.problem_definition.add_task_solution(task, t_start, t_end, assignment)
                print(task_solution)
            except Exception:
                print(f"Error during solution filling")
                raise Exception(f"{task}, not presence in problem task list")
            problem_solution.append(task_solution)
        return problem_solution

    def add_precedence_constraints(self) -> None:
        for task, precedence_tasks in self.problem_definition.get_precedence_constraints().items():
            for precedence_task in precedence_tasks:
                # print(self.decision_variables["t_start"])
                # print(self.decision_variables["t_end"])
                self.model.addConstr(
                    self.decision_variables["t_start"][task] == self.decision_variables["t_end"][precedence_task])

    def add_problem(self, problem) -> None:
        # self.problem_definition.append(problem)
        self.problem_definition = problem

    def check_feasibility(self) -> bool:
        try:
            self.model.computeIIS()
            if self.model.status == gp.GRB.INFEASIBLE:
                # print(f"Precedence constraints infeasible")
                for constraint in self.model.getConstrs():
                    if constraint.IISConstr:
                        print(f"{self.model.getRow(constraint)} {constraint.Sense} {constraint.RHS}")
                return False
        except gp.GurobiError:
            return True
