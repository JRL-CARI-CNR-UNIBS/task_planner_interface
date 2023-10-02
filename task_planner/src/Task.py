from dataclasses import dataclass, field
from typing import List, Dict, Optional, overload, Tuple
from enum import Enum


@dataclass
class Task:
    id: str
    type: str
    agents: List[str] = field(default=None, init=False)
    agents_constraint: List[str]
    precedence_constraints: List[str]
    soft_precedence_constraints: List[str]

    exp_duration: Dict[str, float] = field(default=None, init=False)
    synergy: Dict[Tuple[str, str], Dict[str, float]] = field(default_factory=dict, init=False)

    def update_duration(self, agent: str, duration: float) -> bool:
        """
        Update the expected duration of a task when executed by an agent.

        Args:
            agent (str): The agent for which the duration is to be updated.
            duration (float): The new expected duration to be assigned to the task.

        Returns:
            bool: True if the update was successful, False otherwise.

        Raises:
            AssertionError: If the list of agents is empty or if the
                            specified agent is not present in the list.
            ValueError: If the duration is not a positive numeric value.
        """
        assert self.agents is not None
        if self.agents is None:
            print(f"Empy agents List")
            return False

        assert agent in self.agents
        if agent not in self.agents:
            print(f"Task: {self.id} has no agent: {agent}")
            return False

        assert duration > 0
        if self.exp_duration is None:
            self.exp_duration = {}
        if duration < 0:
            # raise ValueError("Expected duration must be not negative")
            return False

        self.exp_duration[agent] = duration
        return True

    def update_synergy(self, agent, parallel_agent, synergies: Dict[str, float]) -> None:
        """
        Update the synergy value between two agents for a current task.

        Args:
            agent: The primary agent for which the synergy is being updated.
            parallel_agent: The secondary agent with which the synergy is being updated.
            synergies (Dict[str, float]): A dictionary containing the synergy values for different tasks.

        Returns:
            None

        Raises:
            Exception: If the specified primary agent is not defined for the task.
        """

        if agent not in self.agents:
            raise Exception(f"Agent {agent} is not defined for task: {self.id}")
        if not all(value >= 0 for value in synergies.values()):
            raise ValueError("Synergy values must be non-negative.")

        # TODO: Check that contains all tasks
        self.synergy[(agent, parallel_agent)] = synergies
        # Alternative solutions: pass only one synergy as single dict {name:, synergy}
        #                        pass one synergy object
        #                        pass a synergy objects list

    def update_agents(self, agents: List[str]) -> None:
        """
        Update the list of agents associated with the task.

        Args:
            agents (List[str]): The new list of agents.

        Returns:
            None

        Raises:
            ValueError: If the `agents` parameter is not a list or if any item in the list is not a string.
        """
        if not (isinstance(agents, list) and all(isinstance(agent, str) for agent in agents)):
            return ValueError("All items in the agents list must be strings.")

        self.agents = agents

    def update_agents_constraint(self, agents: List[str]) -> None:
        """
        Updates the agents constraint of the task with the provided list of agents.

        Args:
            agents (List[str]): The list of agents to be set as the new agents constraint.

        Raises:
            ValueError: If the agents argument is not a list or if any item in the list is not a string.

        Returns:
            None
        """

        if not (isinstance(agents, list) and all(isinstance(agent, str) for agent in agents)):
            return ValueError("All items in the agents list must be strings.")
        self.agents_constraint = agents

    def add_agent(self, agent: str) -> None:
        if agent not in self.agents:
            self.agents.append(agent)

    def update_precedence_constraints(self) -> None:
        pass

    @overload
    def get_duration(self) -> Dict[str, float]:
        ...

    @overload
    def get_duration(self, agent: str) -> float:
        ...

    def get_duration(self, *args):
        """
        Retrieves the expected duration based on the provided arguments.

        If no arguments are provided, it returns the expected durations for all agents as a dictionary.
        If a single argument (agent) is provided, it returns the expected duration for that agent as a float.

        Args:
            *args: Variable-length arguments. If a single argument is provided, it is treated as the agent name.

        Returns:
            Dict[str, float]: The expected durations based on the provided arguments.

        Raises:
            NotImplemented: If invalid arguments are provided.

        """
        if len(args) == 1 and isinstance(args[0], str):  # Param: specify agent
            agent = args[0]
            assert agent in self.agents
            if agent not in self.exp_duration.keys():
                print(f"Task: {self.id} has no exp_duration for agent: {agent}")
                raise Exception
            return self.exp_duration[agent]
        if len(args) == 0:
            return self.exp_duration
        raise NotImplementedError("Invalid arguments provided.")


    def get_synergy(self, agent: str, parallel_agent: str, parallel_task: str) -> Optional[str]:
        """
        Retrieves the synergy value based on the provided agent, parallel_agent, and parallel_task.

        :param agent: str, the agent for which to retrieve the synergy value.
        :param parallel_agent: str, the parallel agent involved in the synergy.
        :param parallel_task: str, the specific task for which to retrieve the synergy value.

        :return: Optional[str], the synergy value corresponding to the provided parameters.
                 Returns the synergy value if the combination of agent, parallel_agent, and parallel_task exists
                 in the synergy dictionary. Otherwise, returns None.
        """

        if (agent, parallel_agent) in self.synergy and parallel_task in self.synergy[(agent, parallel_agent)]:
            return self.synergy[(agent, parallel_agent)][parallel_task]
        return None

    def get_synergies(self,
                      agent: Optional[str] = None,
                      parallel_agent: Optional[str] = None) -> Optional[Dict[Tuple[str, str], Dict[str, float]]]:
        """
        Retrieves the synergies based on the provided agent(s) and parallel agent.

        :param agent: Optional[str], the agent for which to retrieve the synergies. Default is None.
        :param parallel_agent: Optional[str], the parallel agent to consider for synergies. Default is None.

        :return: Optional[Dict[Tuple[str, str], Dict[str, float]]], the requested synergies.
                 Returns the entire synergy dictionary if neither agent nor parallel_agent is provided.
                 Returns the synergies for the specified agent with all parallel agents if only agent is provided.
                 Raises a ValueError if only parallel_agent is provided, indicating that agent must also be provided.        """

        if agent is None and parallel_agent is None:
            return self.synergy
        elif agent is not None and parallel_agent is None:
            specified_synergy = \
                {agent: synergies for involved_agents, synergies in self.synergy.items() if involved_agents[0] == agent}
            return specified_synergy
        elif agent is not None and parallel_agent is not None:
            if (agent, parallel_agent) in self.synergy:
                return self.synergy[(agent, parallel_agent)]
            raise KeyError(f"Involved agent: ({agent},{parallel_agent}) not defined in synergy for task: {self.type}")

        else:
            raise ValueError("You must provide both 'agent' and 'parallel_agent'.")

    def get_id(self) -> str:
        return self.id

    def get_type(self) -> str:
        return self.type

    def get_agents(self) -> List[str]:
        return self.agents

    def get_agents_constraint(self) -> List[str]:
        return self.agents_constraint

    def get_precedence_constraints(self) -> Optional[List[str]]:
        return self.precedence_constraints

    def get_soft_constraints(self) -> Optional[List[str]]:
        return self.soft_precedence_constraints

    def check_precedence_constraint(self, task: str) -> bool:
        if task in self.precedence_constraints:
            return True
        return False

    def get_max_duration(self) -> float:
        return max(self.exp_duration.values())

    def get_not_enabled_agents(self) -> Optional[List[str]]:
        not_enabled_agents = []
        if self.agents_constraint:
            not_enabled_agents = list(set(self.agents) - set(self.agents_constraint))
        return not_enabled_agents

    def __eq__(self, other):
        return isinstance(other, Task) and self.id == other.id

    def __hash__(self):
        return hash(self.id)

    def __repr__(self):
        if self.exp_duration:
            return f"{self.id}, {self.exp_duration}"
        else:
            return f"{self.id}"


@dataclass
class TaskSolution:

    task: Task
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

    def get_task(self) -> Task:
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
