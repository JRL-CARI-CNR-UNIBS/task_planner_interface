import rospy
from task import Task, TaskInstance
from dataclasses import dataclass, field
from typing import Dict, Optional, Set
from utils import DataLoadingError

from problem_loader import ProblemLoaderInterface, ProblemLoaderUtility


def validate_ros_goal_format(problem_data):
    if not isinstance(problem_data, list):
        raise ValueError("Yaml fiLE not well formatted: 'goal' must be present and has to be a list.")


@dataclass
class ROSProblemLoader(ProblemLoaderInterface):
    goal_param_name: str = field(default=None, init=True)

    def __post_init__(self):
        if self.goal_param_name is None:
            raise ValueError("Goal param name is None")

    def load_instances(self, tasks_from_knowledge_base: Set[Task]) -> Set[TaskInstance]:
        if not rospy.has_param(self.goal_param_name):
            raise DataLoadingError(f"Goal param: {self.goal_param_name} does not exist")

        raw_problem_goal = rospy.get_param(self.goal_param_name)

        try:
            validate_ros_goal_format(raw_problem_goal)
        except ValueError as exc:
            raise DataLoadingError(exc)

        known_task = dict()
        for task in tasks_from_knowledge_base:
            known_task[task.get_task_name()] = task

        task_instances = set()
        for raw_task_instance in raw_problem_goal:
            try:
                task_instance = ProblemLoaderUtility.get_single_instance(raw_task_instance, known_task)
            except ValueError as exc:
                raise DataLoadingError(exc)

            task_instances.add(task_instance)
        return task_instances
