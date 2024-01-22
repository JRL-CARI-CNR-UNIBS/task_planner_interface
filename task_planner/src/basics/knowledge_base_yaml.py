from knowledge_base import KnowledgeBaseInterface
from dataclasses import dataclass, field

import yaml
from pathlib import Path
from typing import Dict


@dataclass
class YAMLKnowledgeBase(KnowledgeBaseInterface):
    yaml_file_path: Path = field(init=True)
    knowledge_base_data: Dict = field(init=False, default_factory=dict)

    def __post_init__(self):
        try:
            self.load_yaml_file()
            self._validate_format()
        except (yaml.YAMLError, ValueError) as exc:
            raise ValueError(exc)

    def load_yaml_file(self):
        with open(self.yaml_file_path, 'r') as file:
            try:
                self.knowledge_base_data = yaml.safe_load(file)
            except yaml.YAMLError as exc:
                raise exc

    def check_task_existence(self, task_name: str):
        for task in self.knowledge_base_data['task_properties']:
            if task["name"] == task_name:
                return True
        return False

    def get_task_agents(self, task_name: str):
        if task_name in self.knowledge_base_data:
            return list(self.knowledge_base_data[task_name]['expected_duration'].keys())
        return []

    def get_tasks_stats(self, task_name: str):
        if task_name in self.knowledge_base_data:
            return self.knowledge_base_data[task_name].get('expected_duration', {})
        return {}

    def get_tasks_synergies(self, task_name: str):
        if task_name in self.knowledge_base_data:
            return self.knowledge_base_data[task_name].get('standard_deviation', {})
        return {}

    def check_task_existence(self):
        # Implementazione specifica per YAML
        pass

    def get_task_info(self):
        # Implementazione specifica per YAML
        pass

    def get_tasks_stats(self):
        # Implementazione specifica per YAML
        pass

    def _validate_format(self):
        # Check if 'tasks_properties' and 'task_synergies' are present in the YAML data
        if 'tasks_properties' not in self.knowledge_base_data or 'task_synergies' not in self.knowledge_base_data:
            raise ValueError("Invalid format: 'tasks_properties' and 'task_synergies' are required.")

        # Check if 'tasks_properties' is a list
        if not isinstance(self.knowledge_base_data['tasks_properties'], list):
            raise ValueError("Invalid format: 'tasks_properties' should be a list of task.")

        # Validate each task in 'tasks_properties'
        for task in self.yaml_data['tasks_properties']:
            if not isinstance(task, dict) or 'name' not in task or 'agents' not in task:
                raise ValueError(
                    "Invalid format: Each task in 'tasks_properties' should be a dictionary with 'name' and 'agents'.")

            # Check if 'agents' is a list
            if not isinstance(task['agents'], list):
                raise ValueError("Invalid format: 'agents' in each task should be a list.")

            # Validate each agent in 'agents'
            for agent in task['agents']:
                if not isinstance(agent,
                                  dict) or 'name' not in agent or 'expected_duration' not in agent or 'duration_std_dev' not in agent:
                    raise ValueError(
                        "Invalid format: Each agent in 'agents' should be a "
                        "dictionary with 'name', 'expected_duration', and 'duration_std_dev'.")

        # Check if 'task_synergies' is a list
        if not isinstance(self.knowledge_base_data['task_synergies'], list):
            raise ValueError("Invalid format: 'task_synergies' should be a list of synergies.")
