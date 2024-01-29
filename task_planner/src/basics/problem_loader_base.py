from task import Task, TaskInstance
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Set

from knowledge_base import KnowledgeBaseInterface

from abc import ABC, abstractmethod


@dataclass
class ProblemLoaderBase(ABC):
    @abstractmethod
    def load_instances(self) -> set[TaskInstance]:
        pass

class ROSProblemLoader(ProblemLoaderBase):


