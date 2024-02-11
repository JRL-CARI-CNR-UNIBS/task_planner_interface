import rospy
from task import Task, TaskInstance
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Set

from knowledge_base import KnowledgeBaseInterface

from abc import ABC, abstractmethod


@dataclass
class ProblemLoaderBase(ABC):
    @abstractmethod
    def load_instances(self, tasks_from_knowledge_base: Set[Task]) -> Set[TaskInstance]:
        pass

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


@dataclass
class ROSProblemLoader(ProblemLoaderBase):
    goal_param_name: str = field(default=None, init=True)

    def __post_init__(self):
        if self.goal_param_name is None:
            raise ValueError("Goal param name is None")

    def load_instances(self, tasks_from_knowledge_base: Set[Task]) -> Set[TaskInstance]:
        if not rospy.has_param(self.goal_param_name):
            raise ValueError(f"Goal param: {self.goal_param_name} does not exist")

        raw_problem_goal = rospy.get_param(self.goal_param_name)

        try:
            validate_ros_goal_format(raw_problem_goal)
        except ValueError as exc:
            raise exc

        known_task = dict()
        for task in tasks_from_knowledge_base:
            known_task[task.get_task_name()] = task

        task_instances = set()
        for raw_task_instance in raw_problem_goal:
            try:
                task_instance = get_single_instance(raw_task_instance, known_task)
            except ValueError as exc:
                raise exc

            task_instances.add(task_instance)
        return task_instances


import yaml

GOAL = 'goal'
TASK_NAME_KEY = 'task_name'
PRECEDENCE_CONSTRAINTS_KEY = 'precedence_constraints'
IMMEDIATE_PRECEDENCE_CONSTRAINTS_KEY = 'immediate_precedence_constraints'
REQUIRED_AGENTS_KEY = 'required_agents'


def get_single_instance(raw_task_instance, known_task: Dict[str, Task]) -> TaskInstance:
    if not isinstance(raw_task_instance, dict):
        raise ValueError(f"Invalid format: the single task instance in goal must be dictionary.")
    if len(raw_task_instance.keys()) > 1:
        raise ValueError(f"Invalid format: each task must have one key (name). Keys: {raw_task_instance.keys()}.")

    task_id = list(raw_task_instance.keys())[0]
    task_info = raw_task_instance[task_id]

    if not isinstance(task_info, dict):
        raise ValueError(f"Invalid format: Information regarding a task must be dict.")
    if TASK_NAME_KEY not in task_info:
        raise ValueError(f"Invalid format: task: {task_id} has no task name.")
    task_name = task_info[TASK_NAME_KEY]
    if task_name not in known_task:
        raise ValueError(f"Task id: {task_id}, of type: {task_name}, is not known")
    task = known_task[task_name]

    agent_constraints = set()
    if REQUIRED_AGENTS_KEY in task_info:
        if isinstance(task_info[REQUIRED_AGENTS_KEY], str):
            agent_constraints = {task_info[REQUIRED_AGENTS_KEY]}
        elif isinstance(task_info[REQUIRED_AGENTS_KEY], list):
            agent_constraints = set(task_info[REQUIRED_AGENTS_KEY])
        else:  # Neither str or list (and is setted)
            raise ValueError(f"Invalid format: agents of: {task_id} must be string or list of string.")
    immediate_precedence_constraint = None
    if IMMEDIATE_PRECEDENCE_CONSTRAINTS_KEY in task_info:
        if isinstance(task_info[IMMEDIATE_PRECEDENCE_CONSTRAINTS_KEY], str):
            immediate_precedence_constraint = task_info[IMMEDIATE_PRECEDENCE_CONSTRAINTS_KEY]
        else:  # Neither str (and is setted)
            raise ValueError(
                f"Invalid format: immediate precedence of: {task_id} must be string or list of string.")
    precedence_constraint = set()
    if PRECEDENCE_CONSTRAINTS_KEY in task_info:
        if isinstance(task_info[PRECEDENCE_CONSTRAINTS_KEY], str):
            precedence_constraint = {task_info[PRECEDENCE_CONSTRAINTS_KEY]}
        elif isinstance(task_info[PRECEDENCE_CONSTRAINTS_KEY], list):
            precedence_constraint = set(task_info[PRECEDENCE_CONSTRAINTS_KEY])
        else:  # Neither str or list (and is setted)
            raise ValueError(f"Invalid format: precedence of: {task_id} must be string or list of string.")

    try:
        task_instance = TaskInstance(id=task_id,
                                     task=task,
                                     agents_constraints=agent_constraints,
                                     immediate_precedence_constraint=immediate_precedence_constraint,
                                     precedence_constraints=precedence_constraint)
    except ValueError as exc:
        raise ValueError
    return task_instance


def validate_yaml_goal_format(problem_data):
    if GOAL not in problem_data or not isinstance(problem_data[GOAL], list):
        raise ValueError("Yaml fiLE not well formatted: 'goal' must be present and has to be a list.")


def validate_ros_goal_format(problem_data):
    if not isinstance(problem_data, list):
        raise ValueError("Yaml fiLE not well formatted: 'goal' must be present and has to be a list.")


@dataclass
class YAMLProblemLoader(ProblemLoaderBase):
    yaml_file_path: str = field(default=None, init=True)

    def load_instances(self, tasks_from_knowledge_base: Set[Task]) -> Set[TaskInstance]:
        try:
            raw_problem_goal = self._load_yaml_file()
        except (FileNotFoundError, yaml.YAMLError) as exc:
            raise exc

        try:
            validate_yaml_goal_format(raw_problem_goal)
        except ValueError as exc:
            raise exc

        known_task = dict()
        for task in tasks_from_knowledge_base:
            known_task[task.get_task_name()] = task

        task_instances = set()
        for raw_task_instance in raw_problem_goal[GOAL]:
            try:
                task_instance = get_single_instance(raw_task_instance, known_task)
            except ValueError as exc:
                raise exc

            task_instances.add(task_instance)
        return task_instances

    def _load_yaml_file(self):
        try:
            with open(self.yaml_file_path, 'r') as file:
                try:
                    problem_data = yaml.safe_load(file)
                except yaml.YAMLError as exc:
                    raise exc
        except FileNotFoundError as exc:
            raise exc
        return problem_data
