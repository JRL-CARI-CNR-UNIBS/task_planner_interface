import rospy
from task import Task, TaskInstance
from dataclasses import dataclass, field
from typing import Dict, Optional, Set
from utils import DataLoadingError

from abc import ABC, abstractmethod

TASK_NAME_KEY = 'task_name'
PRECEDENCE_CONSTRAINTS_KEY = 'precedence_constraints'
IMMEDIATE_PRECEDENCE_CONSTRAINTS_KEY = 'immediate_precedence_constraints'
REQUIRED_AGENTS_KEY = 'required_agents'


@dataclass
class ProblemLoaderInterface(ABC):
    @abstractmethod
    def load_instances(self, tasks_from_knowledge_base: Set[Task]) -> Set[TaskInstance]:
        pass


@dataclass
class ProblemLoaderUtility:

    @staticmethod
    def get_agent_constraint(task_info: Dict) -> Set[str]:
        agent_constraints = set()

        raw_task_agent_info = task_info[REQUIRED_AGENTS_KEY]
        if REQUIRED_AGENTS_KEY in task_info:
            if isinstance(raw_task_agent_info, str):
                agent_constraints = {raw_task_agent_info}
            elif isinstance(raw_task_agent_info, list) and all(isinstance(agent, str) for agent in raw_task_agent_info):
                agent_constraints = set(raw_task_agent_info)
            else:  # Neither str or list (and is setted)
                raise ValueError(f"Invalid format of Agents: it must be string or list of string.")
        return agent_constraints

    @staticmethod
    def get_immediate_precedence_constraints(task_info: Dict) -> Optional[str]:
        immediate_precedence_constraint = None
        raw_ip_constraints = task_info[IMMEDIATE_PRECEDENCE_CONSTRAINTS_KEY]
        if IMMEDIATE_PRECEDENCE_CONSTRAINTS_KEY in task_info:
            if isinstance(raw_ip_constraints, str) and raw_ip_constraints != "":  # string
                immediate_precedence_constraint = raw_ip_constraints
            elif isinstance(raw_ip_constraints, str) and raw_ip_constraints == "":  # empty string
                immediate_precedence_constraint = None
            elif isinstance(raw_ip_constraints, list):  # one element list
                if len(raw_ip_constraints) > 1:  # more than one element -> wrong
                    raise ValueError(
                        f"Invalid format of immediate precedence: "
                        f"if it is a list must have just one element.")
                if isinstance(raw_ip_constraints[0], str):  # string
                    immediate_precedence_constraint = raw_ip_constraints[0]
                else:
                    raise ValueError(
                        f"Invalid format of immediate precedence: "
                        f"the element in the list must be a string.")
            else:  # Neither str/single element list (and is set)
                raise ValueError(
                    f"Invalid format of immediate precedence: "
                    f"it must be string or list of string.")
        return immediate_precedence_constraint

    @staticmethod
    def get_precedence_constraints(task_info: Dict) -> Set[str]:
        precedence_constraints = set()
        raw_precedence_constraints = task_info[PRECEDENCE_CONSTRAINTS_KEY]
        if PRECEDENCE_CONSTRAINTS_KEY in task_info:
            if isinstance(raw_precedence_constraints, str):
                precedence_constraints = {raw_precedence_constraints}
            elif (isinstance(raw_precedence_constraints, list) and
                  all(isinstance(precedence_constraint, str) for precedence_constraint in raw_precedence_constraints)):
                precedence_constraints = set(raw_precedence_constraints)
            else:  # Neither str or list (and is setted)
                raise ValueError(f"Invalid format: precedence constraints must be string or list of string.")
        return precedence_constraints

    @staticmethod
    def get_single_instance(raw_task_instance: Dict, known_task: Dict[str, Task]) -> TaskInstance:
        if not isinstance(raw_task_instance, dict):
            raise ValueError(f"Invalid format: the single task instance in goal must be dictionary.")
        if len(raw_task_instance.keys()) > 1:
            raise ValueError(
                f"Invalid format: each task must have one key (Task id). Keys: {raw_task_instance.keys()}.")

        task_id = list(raw_task_instance.keys())[0]
        task_info = raw_task_instance[task_id]

        if not isinstance(task_info, dict):
            raise ValueError(f"Invalid format: Information regarding a task must be dict.")
        if TASK_NAME_KEY not in task_info:
            raise ValueError(f"Invalid format: task: {task_id} has no task name.")
        task_name = task_info[TASK_NAME_KEY]
        if task_name not in known_task:
            raise ValueError(f"Task id: {task_id} is of type: {task_name} and is not known.")
        task = known_task[task_name]

        # Retrieve Agents Constraints
        try:
            agent_constraints = ProblemLoaderUtility.get_agent_constraint(task_info)
        except ValueError as exception:
            raise ValueError(f"Error loading task id: {task_id}: {exception}")

        # Retrieve immediate precedence constraints
        try:
            immediate_precedence_constraint = ProblemLoaderUtility.get_immediate_precedence_constraints(task_info)
        except ValueError as exception:
            raise ValueError(f"Error loading task id: {task_id}: {exception}")

        # Retrieve precedence constraints
        try:
            precedence_constraints = ProblemLoaderUtility.get_precedence_constraints(task_info)
        except ValueError as exception:
            raise ValueError(f"Error loading task id: {task_id}: {exception}")

        try:
            task_instance = TaskInstance(id=task_id,
                                         task=task,
                                         agents_constraints=agent_constraints,
                                         immediate_precedence_constraint=immediate_precedence_constraint,
                                         precedence_constraints=precedence_constraints)
        except ValueError:
            raise ValueError
        return task_instance

    #     raw_problem_goal = self._load_raw_goal()
    #     try:
    #         self._validate_raw_goal(raw_problem_goal)
    #     except ValueError as exc:
    #         raise exc
    #
    # @abstractmethod
    # def _load_raw_goal(self):
    #     pass
    #
    # @abstractmethod
    # def _validate_raw_goal(self, raw_problem_goal: Dict[str, List]):
    #     pass
    #
    # def _get_tasks_instances(self, raw_problem_goal: Dict[str, List]) -> set[TaskInstance]:
    #     pass

