from abc import ABC, abstractmethod
from task import TaskStatistics, TaskAgentCorrespondence, TaskSynergies
from typing import Set


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
    def get_task_synergies(self, main_task_name: str, main_agent_name: str) -> TaskSynergies:
        pass


class DataLoadingError(Exception):
    pass
