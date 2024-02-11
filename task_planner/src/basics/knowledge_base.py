from abc import ABC, abstractmethod
from task import TaskStatistics, TaskAgentCorrespondence, TaskSynergies
from typing import Set

DEFAULT_KNOWLEDGE_BASES_LIST = [
    "yaml",
    "ros"
]


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


class KnowledgeBaseFactory:
    def __init__(self):
        self._creators = {}

    def register_format(self, knowledge_base_type: str, creator, *args, **kwargs):
        self._creators[knowledge_base_type] = (creator, args, kwargs)

    def get_knowledge_base(self, knowledge_base_type: str):
        creator_info = self._creators.get(knowledge_base_type)
        if not creator_info:
            raise ValueError(knowledge_base_type)
        creator, args, kwargs = creator_info
        return creator(*args, **kwargs)


# class ObjectFactory:
#     def __init__(self):
#         self._builders = {}
#
#     def register_builder(self, key, builder):
#         self._builders[key] = builder
#
#     def create(self, key, **kwargs):
#         builder = self._builders.get(key)
#         if not builder:
#             raise ValueError(key)
#         return builder(**kwargs)
