from dataclasses import dataclass, field
from typing import Optional, List, Set

from task_planner_interface_msgs.srv import (TaskType, TaskTypeResponse, TasksInformation,
                                             TasksInformationResponse, TasksStatistics)
from task_planner_statistics.srv import TaskSynergies as TaskSynergiesSrv
from task_planner_statistics.srv import TaskSynergiesRequest

import rospy
import rosservice
from knowledge_base import KnowledgeBaseInterface, DataLoadingError
from task import TaskAgentCorrespondence, TaskStatistics, TaskSynergies
from .utils import Color, UserMessages, Statistics

TIMEOUT = 10


@dataclass
class ROSKnowledgeBase(KnowledgeBaseInterface):
    task_type_check_srv_name: str = field(init=True)
    task_agents_srv_name: str = field(init=True)
    task_stats_srv_name: str = field(init=True)
    task_synergies_srv_name: Optional[str] = field(default=None, init=True)

    task_type_check_srv: rospy.ServiceProxy = field(init=False)
    task_agents_srv: rospy.ServiceProxy = field(init=False)
    task_stats_srv: rospy.ServiceProxy = field(init=False)
    task_synergies_srv: Optional[rospy.ServiceProxy] = field(default=None, init=False)

    def __post_init__(self):
        service_list = rosservice.get_service_list()
        # Wait services
        for service_name in [self.task_type_check_srv_name,
                             self.task_agents_srv_name,
                             self.task_stats_srv_name,
                             self.task_synergies_srv_name]:
            rospy.loginfo("Waiting {service_name}")
            rospy.wait_for_service(service=service_name, timeout=rospy.Duration(secs=TIMEOUT))
            if service_name not in service_list:
                raise ValueError(f"Service {service_name} not found.")
            rospy.loginfo(f"Service Ready: {service_name}")
        # Instantiate services
        self.task_type_check_srv = rospy.ServiceProxy(self.task_type_check_srv_name, TaskType)
        self.task_agents_srv = rospy.ServiceProxy(self.task_agents_srv_name, TasksInformation)
        self.task_stats_srv = rospy.ServiceProxy(self.task_stats_srv_name, TasksStatistics)
        if self.task_synergies_srv_name:
            self.task_synergies_srv = rospy.ServiceProxy(self.task_synergies_srv_name, TaskSynergiesSrv)

    def check_task_existence(self, task_name: str) -> bool:
        try:
            task_check_result: TaskTypeResponse
            task_check_result = self.task_type_check_srv(task_name)
            if not task_check_result.exist:
                rospy.logerr(UserMessages.TASK_NOT_PRESENT.value.format(task_name))
                return False
            return True
        except rospy.ServiceException as e:
            rospy.logerr(UserMessages.SERVICE_FAILED.value.format(self.task_type_check_srv_name))
            raise DataLoadingError(UserMessages.SERVICE_FAILED.value.format(self.task_type_check_srv_name))

    def get_task_agents(self) -> Set[TaskAgentCorrespondence]:
        try:
            tasks_agents_result: TasksInformationResponse
            tasks_agents_result = self.task_agents_srv()
            tasks_agents_correspondence = set()
            for task_info in tasks_agents_result.tasks_info:
                task_agent_correspondence = TaskAgentCorrespondence(task_name=task_info.name, agents=task_info.agents)
                if task_agent_correspondence in tasks_agents_correspondence:  # same task name
                    print(f"Warning, task agents info of: {task_info.name} duplicated, old removed")
                tasks_agents_correspondence.add(task_agent_correspondence)
            return tasks_agents_correspondence
        except rospy.ServiceException:
            rospy.logerr(UserMessages.SERVICE_FAILED.value.format(self.task_agents_srv_name))
            raise DataLoadingError(UserMessages.SERVICE_FAILED.value.format(self.task_agents_srv_name))

    def get_tasks_stats(self) -> Set[TaskStatistics]:
        try:
            tasks_stats_result: TasksStatistics
            tasks_stats_result = self.task_stats_srv()
            tasks_stats = set()
            for task_stats in tasks_stats_result.statistics:
                task_stats_obj = TaskStatistics(task_name=task_stats.name,
                                                agent_name=task_stats.agent,
                                                statistics=Statistics(expected_duration=task_stats.exp_duration,
                                                                      duration_std_dev=task_stats.duration_std))
                if task_stats_obj in tasks_stats:   # same name and agent
                    rospy.loginfo(UserMessages.CUSTOM_ORANGE.value.format
                                  (f"Warning, task stats of: {task_stats.name} duplicated, old removed"))
                tasks_stats.add(task_stats_obj)
            return tasks_stats
        except rospy.ServiceException as e:
            rospy.logerr(UserMessages.SERVICE_FAILED.value.format(self.task_stats_srv_name))
            raise DataLoadingError(UserMessages.SERVICE_FAILED.value.format(self.task_agents_srv_name))

    def get_task_stats(self, task_name: str, agent_name: str) -> Statistics:
        pass
        # try:
        #     tasks_stats_result: TasksStatistics
        #     tasks_stats_result = self.get_tasks_stats_srv()
        #     tasks_stats = set()
        #     for task_stats in tasks_stats_result.statistics:
        #         task_stats_obj = TaskStatistics(task_name=task_name,
        #                                         agent_name=agent_name,
        #                                         statistics=Statistics(expected_duration=task_stats.exp_duration,
        #                                                               duration_std_dev=task_stats.duration_std))
        #         if task_stats_obj in tasks_stats:
        #             print(f"Warning, task stats of: {task_stats.name} duplicated")
        #         tasks_stats.add(task_stats_obj)
        #     return tasks_stats
        # except rospy.ServiceException as e:
        #     rospy.logerr(UserMessages.SERVICE_FAILED.value.format(self.task_stats_srv_name))
        #     raise DataLoadingError(UserMessages.SERVICE_FAILED.value.format(self.task_agents_srv_name))

    def get_tasks_synergies(self, task_name: str, agent: str) -> TaskSynergies:
        """

        Args:
            task_name: Main task name
            agent:  Main agent name

        Returns: Return the TaskSynergies object of that task and agent with the respect with all the other task-agents

        """
        task_synergies_request = TaskSynergiesRequest()
        task_synergies_request.task_name = task_name
        task_synergies_request.agent = agent

        task_synergies_obj = TaskSynergies(task_name, agent)
        try:
            task_synergies_result = self.task_synergies_srv(task_synergies_request)
            for task_synergies in task_synergies_result.synergies:
                try:
                    task_synergies_obj.add_synergy(other_task_name=task_synergies.task_name,
                                                   other_agent_name=task_synergies.agent,
                                                   synergy_value=task_synergies.synergy,
                                                   std_dev=task_synergies.std_err)
                except ValueError as exception:
                    raise DataLoadingError(exception)
        except rospy.ServiceException as e:
            rospy.logerr(UserMessages.SERVICE_FAILED.value.format(self.task_stats_srv_name))
            raise DataLoadingError(UserMessages.SERVICE_FAILED.value.format(self.task_agents_srv_name))
        return task_synergies_obj
