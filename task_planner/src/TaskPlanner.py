from Problem import Problem
from Task import TaskSolution

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple

import gurobipy as gp
import itertools
import os
from utils import Objective

EPS = 1E-6
BIG_M = 1E7


@dataclass
class TaskPlanner:
    name: str
    problem_definition: Problem
    model: gp.Model = field(init=False)
    objective: Objective = field(default=Objective.MAKESPAN)
    n_solutions: float = field(default=1)
    gap: float = field(default=0)
    decision_variables: Dict[str, gp.tupledict] = field(init=False)

    def __post_init__(self):
        """
        The methos make sure that the TaskPlanner if is correctly defined: consistency check, requested solution >
        and if is able to solve respect that objective Returns:

        """
        if self.problem_definition.consistency_check() is not True:
            raise ValueError("The problem is not consistent. Check it!")
        if self.n_solutions < 1:
            raise ValueError("Unable to specify a number of solutions less than 1")
        if not isinstance(self.objective, Objective):
            raise ValueError("Objective should be an instance of Objective class")

    def initialize(self) -> None:
        """
        This method initialize the TaskPlanner Object:
        Retrieve the licence if it exists,
        create the Model,
        define the solution number.

        """
        e = gp.Env(empty=True)
        if True:
            wls_access_id = os.getenv('WLSACCESSID')
            wls_secret = os.environ.get('WLSSECRET')
            license_id = os.environ.get('LICENSEID')

            if wls_access_id is not None and wls_secret is not None and license_id is not None:
                e.setParam('WLSACCESSID', wls_access_id)
                e.setParam('WLSSECRET', wls_secret)
                e.setParam('LICENSEID', int(license_id))
        e.start()
        # Create the model within the Gurobi environment
        self.model = gp.Model(self.name, env=e, )
        # Set the model able to find more than one solution
        if self.n_solutions == 1:
            self.model.setParam("PoolSearchMode", 0)
        else:
            self.model.setParam("PoolSearchMode", 1) # ERA 2

            self.model.setParam("PoolSolutions", self.n_solutions)
        # self.model.setParam("IntFeasTol", 1e-3)
        # self.model.setParam("MIPgap", 0.57)
        if self.gap > 0:
            self.model.setParam("MIPGap", self.gap)
        self.decision_variables = {}

    def create_model(self) -> None:
        """
        This method defines the decision variables and creates the basic model (t_end constraint and assignment constraint).

        Returns: None

        """
        agent_task_combination, cost = gp.multidict(self.problem_definition.get_combinations())
        tasks_list = self.problem_definition.get_tasks_list()

        upper_bound = self.problem_definition.get_nominal_upper_bound() * 10

        # TODO: Ragionare sulle possibili combination
        tasks_per_agent = self.problem_definition.get_tasks_per_agent()

        couple_of_tasks_per_agent = \
            [list(itertools.product(itertools.combinations(tasks_per_agent[agent], 2), [agent])) for agent in
             tasks_per_agent.keys()]
        couple_of_tasks_per_agent = [item for sublist in couple_of_tasks_per_agent for item in sublist]

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

    def add_assignment_constraints(self, tasks_list: List[str]):
        """
        This method adds assignment constraints to the model.

        Args:
            tasks_list: List[str] of tasks 

        Returns:

        """
        # Unique task-agent assignment
        self.model.addConstrs(
            (self.decision_variables["assignment"].sum('*', task) == 1 for task in tasks_list),
            name='unique_assignment')

        # Assignments constraints
        for task, not_enabled_agents in self.problem_definition.get_not_enabled_agents_constraints().items():
            # print(task, not_enabled_agents)
            for agent in not_enabled_agents:
                self.model.addConstr(self.decision_variables["assignment"][(agent, task)] == 0,
                                     name=f'not_enabled_assignment_{task}')

    def add_t_end_constraints(self,
                              agent_task_combination: Dict[Tuple[str, str], float],
                              cost: Dict[Tuple[str, str], float]) -> None:

        # Tend constraints
        t_end = {}
        # print(agent_task_combination)
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

    def add_precedence_constraints(self) -> None:
        for task, precedence_tasks in self.problem_definition.get_precedence_constraints().items():
            for precedence_task in precedence_tasks:
                self.model.addConstr(
                    self.decision_variables["t_start"][task] == self.decision_variables["t_end"][precedence_task])

    def add_soft_precedence_constraints(self) -> None:
        for task, soft_precedence_tasks in self.problem_definition.get_soft_precedence_constraints().items():
            for precedence_task in soft_precedence_tasks:
                self.model.addConstr(
                    self.decision_variables["t_start"][task] >= self.decision_variables["t_end"][precedence_task])

    def add_problem(self, problem: Problem) -> None:
        # self.problem_definition.append(problem)
        self.problem_definition = problem

    def check_feasibility(self) -> bool:
        try:
            self.model.computeIIS()
            if self.model.status == gp.GRB.INFEASIBLE:
                for constraint in self.model.getConstrs():
                    if constraint.IISConstr:
                        print(f"{self.model.getRow(constraint)} {constraint.Sense} {constraint.RHS}")
                return False
        except gp.GurobiError:
            return True

    def set_objective(self) -> None:
        cost_function = self.model.addVar(name="J", vtype=gp.GRB.CONTINUOUS)

        if self.objective == Objective.SUM_T_START_END:
            self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["t_end"]) +
                                 gp.quicksum(self.decision_variables["t_start"]))
        elif self.objective == Objective.MAKESPAN:
            self.model.addConstr(cost_function == gp.max_(self.decision_variables["t_end"]))
        elif self.objective == Objective.SUM_T_START:
            self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["t_start"]))
        elif self.objective == Objective.SUM_T_END:
            self.model.addConstr(cost_function == gp.quicksum(self.decision_variables["t_end"]))
        else:
            raise NotImplementedError("Not implemented in base class")
        # Objective function
        self.model.setObjective(cost_function, gp.GRB.MINIMIZE)

    def solve(self) -> None:
        # Optimization
        # self.model.params.NonConvex = 2
        # self.model.optimize()
        # self.model.setParam("TuneTimeLimit", 320)
        # self.model.tune()
        # print(" ------------------ TUNING RESULT -----------------------")
        #
        # for i in range(self.model.tuneResultCount):
        #     self.model.getTuneResult(i)
        #     print(self.model.getTuneResult(i))
        #     self.model.write('tune' + str(i) + '.prm')

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

    def get_solution(self, solution_number: int = 0) -> List[TaskSolution]:
        for v in self.model.getVars():
            if "sigma_index" in v.varName:
                # if "t_end" in v.varName or "t_start" in v.varName:
                print(v.varName, v.x)

        if solution_number > self.n_solutions:
            raise ValueError(f"The solution number must be less than {self.n_solutions}")
        # Set the solution
        # print(solution_number)
        if solution_number >= self.model.getAttr("SolCount"):
            raise ValueError(f"Solver found: {self.model.getAttr('SolCount')} solution")
        self.model.setParam(gp.GRB.Param.SolutionNumber, solution_number)

        agents = self.problem_definition.get_agents()
        task_lists = self.problem_definition.get_tasks_list()
        problem_solution = []
        for task in task_lists:

            t_start = self.decision_variables["t_start"][task].Xn
            t_end = self.decision_variables["t_end"][task].Xn
            # assignments = self.decision_variables["assignment"].select("*", task)

            assignment = None
            for agent in agents:
                decision_variable = self.decision_variables["assignment"].select(agent, task)
                if decision_variable:
                    assert len(decision_variable) == 1
                    if len(decision_variable) == 1 and decision_variable[0].Xn == 1:
                        # if len(decision_variable) == 1 and decision_variable[0].X == 1:
                        assignment = agent
            try:
                task_solution = self.problem_definition.add_task_solution(task, t_start, t_end, assignment)
                # print(task_solution)
            except ValueError:
                print(f"Error during solution filling")
                raise Exception(f"{task} has not valid t_start or t_end")
            except Exception:
                print(f"Error during solution filling")
                raise Exception(f"{task}, not presence in problem task list")
            problem_solution.append(task_solution)
        return problem_solution

    def save_model_to_file(self):
        self.model.write('Model.lp')

    def callback(self, model, where):
        pass

    def compute_synergy_val(self, solution):
        solution.sort(key=lambda task_sol: task_sol.get_start_time())

        problem_solution_agent = {agent: list(filter(lambda task_solution: task_solution.get_assignment()
                                                                           == agent, solution)) for agent in
                                  ["ur5_on_guide", "human_right_arm"]}

        synergy_tot = 0.0
        for main_agent_task_sol in problem_solution_agent["ur5_on_guide"]:
            main_agent_task_sol: TaskSolution

            for parallel_agent_task_sol in problem_solution_agent["human_right_arm"]:
                parallel_agent_task_sol: TaskSolution
                overlapping = min(parallel_agent_task_sol.get_end_time(), main_agent_task_sol.get_end_time()) - max(
                    parallel_agent_task_sol.get_start_time(), main_agent_task_sol.get_start_time())
                if overlapping <= 0:
                    overlapping = 0
                synergy = main_agent_task_sol.get_task().get_synergy("ur5_on_guide", "human_right_arm",
                                                                     parallel_agent_task_sol.get_task().get_type())
                synergy_index = overlapping * (synergy - 1)
                synergy_tot += synergy_index

        # print(f"Synergy tot: {synergy_tot}")
        return synergy_tot
