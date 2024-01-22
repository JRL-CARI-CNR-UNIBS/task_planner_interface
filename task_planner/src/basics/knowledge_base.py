from abc import ABC, abstractmethod
from task import TaskStatistics, TaskAgentCorrespondence, TaskSynergies
from typing import Set

TIMEOUT = 2


class KnowledgeBaseInterface(ABC):
    @abstractmethod
    def check_task_existence(self, task_name: str) -> bool:
        pass

    @abstractmethod
    def get_task_agents(self) -> Set[TaskAgentCorrespondence]:
        pass

    @abstractmethod
    def get_tasks_stats(self) -> Set[TaskStatistics]:
        pass

    @abstractmethod
    def get_tasks_synergies(self, task_name: str, agent: str) -> TaskSynergies:
        pass


class DataLoadingError(Exception):
    pass


# Implementazione di un accesso alla Knowledge Base tramite YAML
import yaml


@dataclass
class YAMLKnowledgeBase(KnowledgeBaseInterface):
    def __init__(self, yaml_file_path):
        self.yaml_file_path = yaml_file_path
        self.knowledge_base_data = self.load_yaml_file()

    def load_yaml_file(self):
        with open(self.yaml_file_path, 'r') as file:
            return yaml.safe_load(file)

    def check_task_existence(self, task_name: str):
        return task_name in self.knowledge_base_data

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
