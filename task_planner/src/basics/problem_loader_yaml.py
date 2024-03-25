from task import Task, TaskInstance
from dataclasses import dataclass, field
from typing import Dict, Optional, Set
from utils import DataLoadingError
import yaml

from problem_loader import ProblemLoaderInterface, ProblemLoaderUtility

GOAL = 'goal'


def validate_yaml_goal_format(problem_data):
    if GOAL not in problem_data or not isinstance(problem_data[GOAL], list):
        raise ValueError("Yaml fiLE not well formatted: 'goal' must be present and has to be a list.")


@dataclass
class YAMLProblemLoader(ProblemLoaderInterface):
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
                task_instance = ProblemLoaderUtility.get_single_instance(raw_task_instance, known_task)
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
