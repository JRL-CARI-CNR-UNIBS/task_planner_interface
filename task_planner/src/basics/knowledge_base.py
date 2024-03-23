from abc import ABC, abstractmethod
from task import TaskStatistics, TaskAgentCorrespondence, TaskSynergies
from typing import Set
from task import Task
from utils import DataLoadingError

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

    def get_tasks(self) -> Set[Task]:
        task_agents_correspondence: Set[TaskAgentCorrespondence]
        try:
            task_agents_correspondence = self.get_task_agents()
        except DataLoadingError as exc:
            raise exc

        if not task_agents_correspondence:
            return DataLoadingError("Empty Knowledge Base")

        task_set = set()
        for task_info in task_agents_correspondence:
            task_name = task_info.get_task_name()
            agents = task_info.get_agents()

            task_obj = Task(task_name=task_name, agents=agents)

            if task_obj in task_set:
                print(f"Warning: {task_name} already present in tasks set.")
            task_set.add(task_obj)
        return task_set


class KnowledgeBaseCreationError(Exception):
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
