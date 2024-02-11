from knowledge_base import KnowledgeBaseInterface, DataLoadingError
from dataclasses import dataclass, field
from typing import Set, Optional

import yaml
from pathlib import Path
from typing import Dict

from task import TaskAgentCorrespondence, TaskStatistics, TaskSynergies
from utils import UserMessages, Statistics

TASK_NAME = 'task_name'
AGENT_NAME = 'agent_name'
EXPECTED_DURATION = 'expected_duration'
DURATION_STD = 'duration_std'
TASK_PROPERTIES = 'tasks_properties'
AGENTS_INFO = 'agents'
TASK_SYNERGIES = 'task_synergies'

# Synergy Keys
MAIN_TASK = 'main_task'
MAIN_AGENT = 'main_agent'
PARALLEL_TASKS = 'parallel_tasks'
PARALLEL_TASK = 'parallel_task'
PARALLEL_AGENT = 'parallel_agent'
SYNERGY_VALUE = 'synergy_value'
SYNERGY_STD = 'synergy_std'


@dataclass
class YAMLKnowledgeBase(KnowledgeBaseInterface):
    yaml_file_path: Path = field(init=True)
    knowledge_base_data: Dict = field(init=False, default_factory=dict)

    def __post_init__(self):
        if self.yaml_file_path.suffix != '.yaml':
            raise ValueError(f"File path: {self.yaml_file_path} is not a yaml")

        try:
            self.load_yaml_file()
            self._validate_format()
        except (FileNotFoundError, yaml.YAMLError, ValueError) as exc:
            raise ValueError(exc)

    def load_yaml_file(self):
        try:
            with open(self.yaml_file_path, 'r') as file:
                try:
                    self.knowledge_base_data = yaml.safe_load(file)
                except yaml.YAMLError as exc:
                    raise exc
        except FileNotFoundError as exc:
            raise exc

    def check_task_existence(self, task_name: str) -> bool:
        for task in self.knowledge_base_data[TASK_PROPERTIES]:
            if task[TASK_NAME] == task_name:
                return True
        print(UserMessages.TASK_NOT_PRESENT.value.format(task_name))
        return False

    def get_task_agents(self) -> Set[TaskAgentCorrespondence]:
        tasks_agents_correspondence = set()
        for task_info in self.knowledge_base_data[TASK_PROPERTIES]:
            task_name = task_info[TASK_NAME]
            single_task_agents = TaskAgentCorrespondence(task_name, set())
            for agent_info in task_info[AGENTS_INFO]:
                single_task_agents.add_agent(agent_info[AGENT_NAME])
            if single_task_agents in tasks_agents_correspondence:  # same task name
                print(f"Warning, task agents info of: {task_info.name} duplicated, old removed")
            tasks_agents_correspondence.add(single_task_agents)
        return tasks_agents_correspondence

    def get_tasks_stats(self) -> Set[TaskStatistics]:
        tasks_stats = set()
        for task_info in self.knowledge_base_data[TASK_PROPERTIES]:
            task_name = task_info[TASK_NAME]
            for agent_info in task_info[AGENTS_INFO]:
                agent_name = agent_info[AGENT_NAME]
                expected_duration = agent_info[EXPECTED_DURATION]
                duration_std = agent_info[DURATION_STD]
                single_task_stats_obj = TaskStatistics(task_name=task_name,
                                                       agent_name=agent_name,
                                                       statistics=Statistics(
                                                           expected_duration=expected_duration,
                                                           duration_std_dev=duration_std))
                if single_task_stats_obj in tasks_stats:  # same name and agent
                    print(UserMessages.CUSTOM_ORANGE.value.format
                          (f"Warning, task stats of: {task_name}, agent: {agent_name} duplicated, old removed"))
                tasks_stats.add(single_task_stats_obj)
        return tasks_stats

    def get_task_synergies(self, main_task_name: str, main_agent_name: str) -> Optional[TaskSynergies]:
        """

        Args:
            main_task_name: Main task name
            main_agent_name:  Main agent name

        Returns: Return the TaskSynergies object of that task and agent with the respect with all the other task-agents

        """
        if not TASK_SYNERGIES in self.knowledge_base_data:
            return None
        task_synergies_obj = TaskSynergies(main_task_name, main_agent_name)
        for task_synergy in self.knowledge_base_data[TASK_SYNERGIES]:
            if task_synergy[MAIN_TASK] != main_task_name or task_synergy[MAIN_AGENT] != main_agent_name:
                continue
            for parallel_task_syn in task_synergy[PARALLEL_TASKS]:
                parallel_task_name = parallel_task_syn[PARALLEL_TASK]
                parallel_agent_name = parallel_task_syn[PARALLEL_AGENT]
                synergy_val = parallel_task_syn[SYNERGY_VALUE]
                synergy_std = parallel_task_syn.get(SYNERGY_STD, None)
                try:
                    task_synergies_obj.add_synergy(other_task_name=parallel_task_name,
                                                   other_agent_name=parallel_agent_name,
                                                   synergy_value=synergy_val,
                                                   std_dev=synergy_std)
                except ValueError as exception:
                    raise DataLoadingError(exception)

        return task_synergies_obj

    def _validate_format(self):
        # TODO: Synergy can also not be there se uno non vuole
        # Check if 'tasks_properties' and 'task_synergies' are present in the YAML data
        if TASK_PROPERTIES not in self.knowledge_base_data or TASK_SYNERGIES not in self.knowledge_base_data:
            raise ValueError(f"Invalid format: '{TASK_PROPERTIES}' and '{TASK_SYNERGIES}' are required.")

        # Check if 'tasks_properties' is a list
        if not isinstance(self.knowledge_base_data[TASK_PROPERTIES], list):
            raise ValueError(F"Invalid format: '{TASK_PROPERTIES}' should be a list of task.")

        # Validate each task in 'tasks_properties'
        for task in self.knowledge_base_data[TASK_PROPERTIES]:
            if not isinstance(task, dict) or TASK_NAME not in task or AGENTS_INFO not in task:
                raise ValueError(
                    f"Invalid format: Each task in '{TASK_PROPERTIES}' should be a dictionary with '{TASK_NAME}' and '{AGENTS_INFO}'.")

            # Check if 'agents' is a list
            if not isinstance(task[AGENTS_INFO], list):
                raise ValueError(f"Invalid format: '{AGENTS_INFO}' in each task should be a list.")

            # Validate each agent in 'agents'
            for agent in task[AGENTS_INFO]:
                if not isinstance(agent,
                                  dict) or AGENT_NAME not in agent or EXPECTED_DURATION not in agent or DURATION_STD not in agent:
                    print(f"Analyze: {agent} info.")
                    raise ValueError(
                        f"Invalid format: Each agent in '{AGENTS_INFO}' should be a "
                        f"dictionary with '{AGENT_NAME}', '{EXPECTED_DURATION}', and '{DURATION_STD}'.")

        # Check if 'task_synergies' is a list
        if not isinstance(self.knowledge_base_data[TASK_SYNERGIES], list):
            raise ValueError("Invalid format: '{TASK_SYNERGIES}' should be a list of synergies.")

        # Validate each synergy in 'task_synergies'
        for synergy in self.knowledge_base_data[TASK_SYNERGIES]:
            if not isinstance(synergy,
                              dict) or MAIN_TASK not in synergy or MAIN_AGENT not in synergy or PARALLEL_TASKS not in synergy:
                raise ValueError(
                    f"Invalid format: Each synergy in '{TASK_SYNERGIES}' should be a dictionary with '{MAIN_TASK}', '{MAIN_AGENT}', and '{PARALLEL_TASKS}'."
                )

            # Check if 'parallel_tasks' is a list
            if not isinstance(synergy[PARALLEL_TASKS], list):
                raise ValueError(f"Invalid format: '{PARALLEL_TASKS}' in each synergy should be a list.")

            # Validate each parallel task in 'parallel_tasks'
            for parallel_task in synergy[PARALLEL_TASKS]:
                if not isinstance(parallel_task,
                                  dict) or PARALLEL_TASK not in parallel_task or PARALLEL_AGENT not in parallel_task or SYNERGY_VALUE not in parallel_task or SYNERGY_STD not in parallel_task:
                    raise ValueError(
                        f"Invalid format: Each parallel task in '{PARALLEL_TASKS}' should be a dictionary with '{PARALLEL_TASK}', '{PARALLEL_AGENT}', '{SYNERGY_VALUE}', and '{SYNERGY_STD}'."
                    )
