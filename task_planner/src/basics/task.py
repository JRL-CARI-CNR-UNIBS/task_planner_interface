from dataclasses import dataclass, field
from typing import List, Dict, Optional, overload, Tuple, Set
from enum import Enum
from statistics_utils import AgentStats, AgentSynergy, TaskSynergies
from multipledispatch import dispatch


# @dataclass
# class TaskInstance:
#     task_id: str
#     task: Task
#
#     agents_constraints: Set[str]
#
#     immediate_precedence_constraint: str
#     precedence_constraints: Set[str]
#
#     def __hash__(self):
#         return hash(self.id)

# @dataclass
# class AgentStats:
#     agent_name: str
#     statistics: Statistics
#
#     def __hash__(self):
#         return hash(self.agent_name)


@dataclass
class Task:
    task_name: str  # type
    agents: Set[str] = field(default_factory=set, init=False)

    statistics: Set[AgentStats] = field(default_factory=set, init=False) # Erano optional
    synergies: Set[AgentSynergy] = field(default_factory=set, init=False)

    def get_task_name(self) -> str:
        return self.task_name

    def get_agents(self) -> Set[str]:
        return self.agents

    def add_agent_statistics(self, agent_statistics: AgentStats):
        if agent_statistics in self.statistics:
            print("Warning: Statistics already present. Updating the old one")
            self.statistics.remove(agent_statistics)
        if agent_statistics.get_agent_name() not in self.agents:
            print(f"Statistics not added! {agent_statistics.get_agent_name()} not in agents of {self.agents}")
            return
        self.statistics.add(agent_statistics)


    # def add_agent_statistics(self, agent_statistics: AgentStats):
    #     if agent_statistics in self.statistics:
    #         print("Warning: Statistics already present")
    #     if agent_statistics.get_agent_name() not in self.agents:
    #         print(f"Statistics not added! {agent_statistics.get_agent_name()} not in agents of {self.agents}")
    #         return
    #     self.statistics.add(agent_statistics)

    # def update_agent_statistics(self, agent_statistics: AgentStats):
    #     if agent_statistics.get_agent_name() not in self.agents:
    #         print(f"Warning: Agent ({agent_statistics.agent_name}) not in task: f{self.task_name}")
    #     if agent_statistics in self.statistics:
    #         self.statistics.remove(agent_statistics)
    #         self.statistics.add(agent_statistics)

    def add_agent(self, agent: str) -> None:
        if agent not in self.agents:
            self.agents.add(agent)

    def remove_agent(self, agent: str):
        if agent in self.agents:
            self.agents.remove(agent)

    def update_agents(self, agents: Set[str]) -> None:
        # TODO: Da cambiare anche il docstring
        """
        Update the list of agents associated with the task.

        Args:
            agents (List[str]): The new list of agents.

        Returns:
            None

        Raises:
            ValueError: If the `agents` parameter is not a list or if any item in the list is not a string.
        """
        if not (isinstance(agents, set) and all(isinstance(agent, str) for agent in agents)):
            return ValueError("All items in the agents set must be strings.")

        self.agents = agents

    def get_statistics(self) -> Set[AgentStats]:
        return self.statistics

    def get_agent_statistics(self, agent_name: str) -> AgentStats:
        for agent_statistic in self.statistics:
            if agent_statistic.get_agent_name() == agent_name:
                return agent_statistic

    def get_synergies(self) -> Set[AgentSynergy]:
        return self.synergies

    def get_agent_synergies(self, main_agent: str) -> Set[AgentSynergy]:
        if main_agent not in self.agents:
            return None
        return {synergy.get_synergy() for synergy in self.synergies if synergy.main_agent_name == main_agent}

    def get_synergies_between_agents(self, main_agent: str, parallel_agent: str) -> Set[AgentSynergy]:
        if main_agent not in self.agents:
            return None
        return {synergy.get_synergy() for synergy in self.synergies if
                synergy.main_agent_name == main_agent and synergy.get_parallel_agent_name() == parallel_agent}

    @dispatch
    def add_synergy(self, agent_synergy: AgentSynergy):
        if agent_synergy.get_main_agent() not in self.agents:
            print(f"Warning: Synergy main agent not in task agents: {self.agents}")
            return
        if agent_synergy in self.synergies:
            self.synergies.remove(agent_synergy) # Aggiunto per avere add_synergy che fa anche update
            print("Warning: synergy already present: Neglected.")
        self.synergies.add(agent_synergy)

    @dispatch
    def add_synergy(self, task_synergies: TaskSynergies):
        if task_synergies.get_main_task_name() != self.task_name:
            print(f"Warning: Synergy with main task not equal to: {self.task_name}")
            return
        if task_synergies.get_main_agent_name() not in self.agents:
            print(f"Warning: Synergy main agent not in task agents: {self.agents}")
            return
        agent_synergies = task_synergies.get_agent_synergies()
        for agent_synergy in agent_synergies:
            self.synergies.add(agent_synergy)

    # def update_synergy(self, agent_synergy: AgentSynergy):
    #     if agent_synergy in self.synergies:
    #         self.synergies.remove(agent_synergy)
    #     self.synergies.add(agent_synergy)

    def __eq__(self, other):
        return ((isinstance(other, Task) and self.task_name == other.task_name) or
                (isinstance(other, str) and self.task_name == other))  # TODO: DA TOGLIERE, dove usato?

    def __hash__(self):
        return hash(self.task_name)

    def __repr__(self):
        return f"Task name: {self.task_name}, Agents: {self.agents}"

    # def update_agents_constraint(self, agents: List[str]) -> None:
    #     """
    #     Updates the agents constraint of the task with the provided list of agents.
    #
    #     Args:
    #         agents (List[str]): The list of agents to be set as the new agents constraint.
    #
    #     Raises:
    #         ValueError: If the agents argument is not a list or if any item in the list is not a string.
    #
    #     Returns:
    #         None
    #     """
    #
    #     if not (isinstance(agents, list) and all(isinstance(agent, str) for agent in agents)):
    #         return ValueError("All items in the agents list must be strings.")
    #     self.agents_constraint = agents

    # def __hash__(self):
    #     return hash(self.id)

    # def __repr__(self):
    #     if self.exp_duration:
    #         return f"{self.id}, {self.exp_duration}"
    #     else:
    #         return f"{self.id}"

    # def update_duration(self, agent: str, duration: float) -> bool:
    #     """
    #     Update the expected duration of a task when executed by an agent.
    #
    #     Args:
    #         agent (str): The agent for which the duration is to be updated.
    #         duration (float): The new expected duration to be assigned to the task.
    #
    #     Returns:
    #         bool: True if the update was successful, False otherwise.
    #
    #     Raises:
    #         AssertionError: If the list of agents is empty or if the
    #                         specified agent is not present in the list.
    #         ValueError: If the duration is not a positive numeric value.
    #     """
    #     assert self.agents is not None
    #     if self.agents is None:
    #         print(f"Empy agents List")
    #         return False
    #     assert agent in self.agents
    #     if agent not in self.agents:
    #         print(f"Task: {self.id} has no agent: {agent}")
    #         return False
    #
    #     assert duration > 0
    #     if self.exp_duration is None:
    #         self.exp_duration = {}
    #     if duration < 0:
    #         # raise ValueError("Expected duration must be not negative")
    #         return False
    #
    #     self.exp_duration[agent] = duration
    #     return True

    # def update_synergy(self, agent, parallel_agent, synergies: Dict[str, float]) -> None:
    #     """
    #     Update the synergy value between two agents for a current task.
    #
    #     Args:
    #         agent: The primary agent for which the synergy is being updated.
    #         parallel_agent: The secondary agent with which the synergy is being updated.
    #         synergies (Dict[str, float]): A dictionary containing the synergy values for different tasks.
    #
    #     Returns:
    #         None
    #
    #     Raises:
    #         Exception: If the specified primary agent is not defined for the task.
    #     """
    #     if self.agents is None:
    #         raise Exception("Agents not set")
    #     if agent not in self.agents:
    #         raise Exception(f"Agent {agent} is not defined for task: {self.id}")
    #     if not all(value >= 0 for value in synergies.values()):
    #         raise ValueError("Synergy values must be non-negative.")
    #
    #     # TODO: Check that contains all tasks
    #     self.synergy[(agent, parallel_agent)] = synergies
    #     # Alternative solutions: pass only one synergy as single dict {name:, synergy}
    #     #                        pass one synergy object
    #     #                        pass a synergy objects list
    # def get_agents_constraint(self) -> List[str]:
    #     return self.agents_constraint
    #
    # def get_precedence_constraints(self) -> Optional[List[str]]:
    #     return self.precedence_constraints
    #
    # def get_soft_constraints(self) -> Optional[List[str]]:
    #     return self.soft_precedence_constraints
    #
    # def check_precedence_constraint(self, task: str) -> bool:
    #     if task in self.precedence_constraints:
    #         return True
    #     return False

    # def get_max_duration(self) -> float:
    #     return max(self.exp_duration.values())

    # def get_not_enabled_agents(self) -> Optional[List[str]]:
    #     not_enabled_agents = []
    #     if self.agents_constraint:
    #         not_enabled_agents = list(set(self.agents) - set(self.agents_constraint))
    #     return not_enabled_agents
    # def get_synergies(self,
    #                   agent: Optional[str] = None,
    #                   parallel_agent: Optional[str] = None) -> Optional[Dict[Tuple[str, str], Dict[str, float]]]:
    #     """
    #     Retrieves the synergies based on the provided agent(s) and parallel agent.
    #
    #     :param agent: Optional[str], the agent for which to retrieve the synergies. Default is None.
    #     :param parallel_agent: Optional[str], the parallel agent to consider for synergies. Default is None.
    #
    #     :return: Optional[Dict[Tuple[str, str], Dict[str, float]]], the requested synergies.
    #              Returns the entire synergy dictionary if neither agent nor parallel_agent is provided.
    #              Returns the synergies for the specified agent with all parallel agents if only agent is provided.
    #              Raises a ValueError if only parallel_agent is provided, indicating that agent must also be provided.        """
    #
    #     if agent is None and parallel_agent is None:
    #         return self.synergies
    #     elif agent is not None and parallel_agent is None:
    #
    #         specified_synergy = \
    #             {agent: synergies for involved_agents, synergies in self.synergy.items() if involved_agents[0] == agent}
    #         return specified_synergy
    #     elif agent is not None and parallel_agent is not None:
    #         if (agent, parallel_agent) in self.synergy:
    #             return self.synergy[(agent, parallel_agent)]
    #         raise KeyError(f"Involved agent: ({agent},{parallel_agent}) not defined in synergy for task: {self.type}")
    #
    #     else:
    #         raise ValueError("You must provide both 'agent' and 'parallel_agent'.")

    # def get_id(self) -> str:
    #     return self.id

    # def get_type(self) -> str:
    #     return self.type
    # def get_synergy(self, agent: str, parallel_agent: str, parallel_task: str) -> Optional[str]:
    #     """
    #     Retrieves the synergy value based on the provided agent, parallel_agent, and parallel_task.
    #
    #     :param agent: str, the agent for which to retrieve the synergy value.
    #     :param parallel_agent: str, the parallel agent involved in the synergy.
    #     :param parallel_task: str, the specific task for which to retrieve the synergy value.
    #
    #     :return: Optional[str], the synergy value corresponding to the provided parameters.
    #              Returns the synergy value if the combination of agent, parallel_agent, and parallel_task exists
    #              in the synergy dictionary. Otherwise, returns None.
    #     """
    #
    #     if (agent, parallel_agent) in self.synergy and parallel_task in self.synergy[(agent, parallel_agent)]:
    #         return self.synergy[(agent, parallel_agent)][parallel_task]
    #     return None
    # def update_precedence_constraints(self) -> None:
    #     pass

    # @overload
    # def get_duration(self) -> Dict[str, float]:
    #     ...
    #
    # @overload
    # def get_duration(self, agent: str) -> float:
    #     ...
    #
    # def get_duration(self, *args):
    #     """
    #     Retrieves the expected duration based on the provided arguments.
    #
    #     If no arguments are provided, it returns the expected durations for all agents as a dictionary.
    #     If a single argument (agent) is provided, it returns the expected duration for that agent as a float.
    #
    #     Args:
    #         *args: Variable-length arguments. If a single argument is provided, it is treated as the agent name.
    #
    #     Returns:
    #         Dict[str, float]: The expected durations based on the provided arguments.
    #
    #     Raises:
    #         NotImplemented: If invalid arguments are provided.
    #
    #     """
    #     if len(args) == 1 and isinstance(args[0], str):  # Param: specify agent
    #         agent = args[0]
    #         assert agent in self.agents
    #         if agent not in self.exp_duration.keys():
    #             print(f"Task: {self.id} has no exp_duration for agent: {agent}")
    #             raise Exception
    #         return self.exp_duration[agent]
    #     if len(args) == 0:
    #         return self.exp_duration
    #     raise NotImplementedError("Invalid arguments provided.")


@dataclass
class TaskInstance:
    id: str = field(init=True)
    task: Task = field(init=True)
    # task: Optional[Task] = field(default=None, init=False)
    # Todo: to reason about init=True, If false then how to fill that field?

    agents_constraints: Set[str] = field(default_factory=set,
                                         init=True)  # TODO: Ha senso un set? Un or tra due agenti? Forse meglio solo 1. Come era
    # TODO: Potrei fare così: se solo 1 forzarlo, se di più metto semplicemente a 0 gli altri in agents di task.
    immediate_precedence_constraint: Optional[str] = field(default=None,
                                                           init=True)  # TODO: Di stringa o di TaskInstance?
    precedence_constraints: Set[str] = field(default_factory=set, init=True)

    def __post_init(self):
        # Check agents constraint in task obj
        for agent in self.agents_constraints:
            if agent not in self.task.get_agents():
                raise ValueError(f"Task id has as agents constraints: {agent} "
                                 f"not present in task: {self.task.get_task_name()} agents.")

    def get_id(self) -> str:
        return self.id

    def get_task(self) -> Task:
        return self.task

    def get_agents_constraints(self) -> Set[str]:
        return self.agents_constraints

    def get_immediate_precedence_constraint(self) -> Optional[str]:
        return self.immediate_precedence_constraint

    def get_precedence_constraints(self) -> Set[str]:
        return self.precedence_constraints

    def check_precedence_constraint(self, task_name: str) -> bool:
        if task_name in self.precedence_constraints:  # TODO: or also immediate precedence?
            return True
        return False

    def update_agents_constraint(self, agents: Set[str]):
        if not (isinstance(agents, Set) and all(isinstance(agent, str) for agent in agents)):
            return ValueError("All items in the agents list must be strings.")
        self.agents_constraints = agents

    def add_agent_constraint(self, agent: str):
        if agent not in self.task.get_agents():
            print(f"Warning: Agent ({agent}) not in Agents of Task: {self.task.get_task_name()}")
        else:
            if agent not in self.agents_constraints:
                self.agents_constraints.add(agent)

    def remove_agent_constraint(self, agent: str):
        if agent in self.agents_constraints:
            self.agents_constraints.remove(agent)
        else:
            print(f"Warning agent: {agent} not in agents constraints of task: {self.task.get_task_name()}")

    def get_not_enabled_agents(self) -> Set[str]:
        # TODO: Maybe better get_unable_agents
        not_enabled_agents = set()
        if self.agents_constraint:
            not_enabled_agents = set(self.task.get_agents()) - set(self.agents_constraint)
        return not_enabled_agents

    def __hash__(self):
        return hash(self.id)

    def __eq__(self, other):
        return isinstance(other, TaskInstance) and other.id == self.id

    def __repr__(self):
        return f"Task instance id: {self.id}, Type: {self.task.get_task_name()}"


@dataclass
class TaskSolution:
    task: TaskInstance
    t_start: float
    t_end: float
    assignment: str

    def __post_init__(self):
        if self.t_end < 0 or self.t_start < -1e-3 or self.t_end < self.t_start:
            raise ValueError("Unfeasible t. start or t. end")

    # def set_start_time(self, t_start: float) -> None:
    #     self.t_start = t_start
    #
    # def set_end_time(self, t_end: float) -> None:
    #     self.t_end = t_end
    #
    # def set_assignment(self, assignment: str) -> None:
    #     self.assignment = assignment

    def get_task(self) -> TaskInstance:
        return self.task

    def get_start_time(self) -> float:
        return self.t_start

    def get_end_time(self) -> float:
        return self.t_end

    def get_assignment(self) -> str:
        return self.assignment

    def __repr__(self):
        return f"{self.task.get_id()}, t_start: {self.t_start}, t_end: {self.t_end}, by: {self.assignment}"


class Status(Enum):
    COMPLETED = "COMPLETED"
    NOT_COMPLETED = "NOT_COMPLETED"


@dataclass
class TaskExecution:
    """Represents the execution of a task.

    Attributes:
        task (Task): The task object associated with this execution.
        t_start (float): The start time of the task execution.
        assignment (str): The assignment identifier for the task execution.
        status (Status, optional): The status of the task execution (default: Status.NOT_COMPLETED).
        t_end (float, optional): The end time of the task execution (default: None).

    Raises:
        ValueError: If the provided t_start or t_end is unfeasible.

    """
    task: Task
    t_start: float
    assignment: str
    status: Status = field(default=Status.NOT_COMPLETED, init=False)
    t_end: Optional[float] = None

    def __post_init__(self):
        """Initializes the TaskExecution object after the dataclass is created.

        Raises:
            ValueError: If the provided t_start or t_end is unfeasible.

        """
        if self.t_start < -1e-3:
            raise ValueError("Unfeasible t. start or t. end")
        if self.t_end is not None:
            if self.t_end < self.t_start or self.t_end < -1e-3:
                raise ValueError(f"Tend must be feasible, task: {self.task.get_id()}")
            self.status = Status.COMPLETED

    def set_as_completed(self, t_end: float):
        """Sets the task execution as completed with the given end time.

        Args:
            t_end (float): The end time of the task execution.

        Raises:
            ValueError: If the provided t_end is unfeasible.

        """
        if t_end < self.t_start or t_end < -1e-3:
            raise ValueError(f"Tend must be feasible, task: {self.task.get_id()}")
        self.t_end = t_end
        self.status = Status.COMPLETED

    def get_task(self) -> Task:
        """Returns the task associated with this execution.

        Returns:
            Task: The task object associated with this execution.

        """
        return self.task


@dataclass
class TaskAgentCorrespondence:
    task_name: str
    agents: Set[str] = field(default_factory=set, init=True)

    def get_task_name(self) -> str:
        return self.task_name

    def get_agents(self) -> Set[str]:
        return self.agents

    def is_agent_capable(self, agent: str) -> bool:
        return agent in self.agents

    def add_agent(self, agent: str):
        if agent in self.agents:
            print(f"Agent: {agent} already present in agents of task: {self.task_name}")
        self.agents.add(agent)

    def __hash__(self):
        return hash(self.task_name)

    def __eq__(self, other):
        return ((isinstance(other, str) and other == self.task_name) or
                (isinstance(other, TaskAgentCorrespondence) and other.task_name == self.task_name))

#     return (isinstance(other, TaskStatistics) and
#             self.task_name == other.task_name and
