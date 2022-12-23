from Task import Task
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
import rospy
from task_planner_interface_msgs.srv import TaskType, TaskTypeResponse, TasksInformation, TasksInformationResponse, \
    TasksStatistics, TasksStatisticsResponse
from utils import *


@dataclass
class Problem:
    task_list: List[Task] = field(default_factory=list, init=False)
    agents: List[str]

    def __post_init__(self):
        # rospy.wait_for_service('mongo_handler/check_task_type')
        # rospy.wait_for_service('mongo_handler/get_task_agents')
        # rospy.wait_for_service('mongo_handler/get_tasks_statistics')
        self.task_check_srv = rospy.ServiceProxy("mongo_handler/check_task_type", TaskType)
        self.get_tasks_info_srv = rospy.ServiceProxy("mongo_handler/get_task_agents", TasksInformation)
        self.get_tasks_stats_srv = rospy.ServiceProxy("mongo_handler/get_tasks_statistics", TasksStatistics)

    def add_task(self, task: Task) -> None:
        self.task_list.append(task)

    def get_task_list_as_dictionary(self) -> Dict[str, Task]:
        return {task.get_id(): task for task in self.task_list}

    def consistency_check(self) -> bool:
        tasks_dict = self.get_task_list_as_dictionary()

        # Check precedence consistency (exists & not-loop)
        for task in self.task_list:
            # if not any(precedence_task in tasks_dict.keys() for precedence_task in task.get_precedence_constraints()):
            #     return False

            for precedence_task in task.get_precedence_constraints():
                # Check if exist all precedence constraints among task_ids
                if precedence_task not in tasks_dict.keys():
                    rospy.loginfo(f"The precedence task: {precedence_task}, of task: {task.get_id()}, does not exist.")
                    return False

                # Check if loop constraints : REMOVED MADE IN Task Planner
            #         if task.get_id() in tasks_dict[precedence_task].get_precedence_constraints():
            #             rospy.loginfo(f"Loop constraints between: {task.get_id()}, and: {task.get_id()}.")
            #             return False
            # # TODO: A after B, B after C, C after A.
            # Check if all task type exist
            try:
                task_check_result = self.task_check_srv(task.get_type())
                if not task_check_result.exist:
                    rospy.loginfo(
                        f"{Color.RED.value} Type {task.get_type()} not present in Knowledge Base {Color.END.value}")
                    return False
            except rospy.ServiceException as e:
                rospy.logerr(UserMessages.SERVICE_FAILED)
                return False
        return True

    def fill_task_agents(self):
        try:
            tasks_info_result = self.get_tasks_info_srv()
        except rospy.ServiceException as e:
            rospy.logerr(UserMessages.SERVICE_FAILED)
            return False

        # Retrieve agents-task ability
        # task_agents_correspondence = {task_info.name: task_info.agents for task_info in tasks_info_result.tasks_info}
        task_agents_correspondence = {}
        for task_info in tasks_info_result.tasks_info:
            task_agents_correspondence[task_info.name] = task_info.agents

        for task in self.task_list:
            if not task.get_agents():  # Empty => Take mongoDB agents
                task.update_agents(task_agents_correspondence[task.get_type()])
            else:
                # Check if the specified agent (in task goal) can actually perform it
                # if not all(required_agent in task_agents_correspondence[task.get_type()] for required_agent in task.get_agents()):
                for required_agent in task.get_agents_constraint():
                    if required_agent not in task_agents_correspondence[task.get_type()]:
                        rospy.loginfo(
                            f"{Color.RED.value}The specified agent: {required_agent} cannot perform task: {task.get_id()} {Color.END.value}")

    def update_tasks_statistics(self):
        try:
            tasks_stats_result = self.get_tasks_stats_srv()
        except rospy.ServiceException as e:
            rospy.logerr(UserMessages.SERVICE_FAILED)
            return False
        # utils: tasks_stasts_dict : {task_name: {agent1: {stats}, agent2: {stats}}}
        tasks_stasts_dict = {}
        for tasks_stats in tasks_stats_result.statistics:
            if tasks_stats.name not in tasks_stasts_dict:
                tasks_stasts_dict[tasks_stats.name] = {}
            if tasks_stats.agent not in tasks_stasts_dict[tasks_stats.name]:
                tasks_stasts_dict[tasks_stats.name][tasks_stats.agent] = {}

            tasks_stasts_dict[tasks_stats.name][tasks_stats.agent] = {"exp_duration": tasks_stats.exp_duration,
                                                                      "duration_std": tasks_stats.duration_std}

        for task in self.task_list:
            for agent in task.get_agents():
                if agent not in tasks_stasts_dict[task.get_type()].keys():
                    print(f"Missing task statistics for: {task}, for agent: {agent}")
                    return False
                task.update_duration(agent, tasks_stasts_dict[task.get_type()][agent]["exp_duration"])
            # Update also statistics for agents present in Knowledge base but not in task-Goal
            # for agent in set(tasks_stasts_dict[task.get_type()].keys()).difference({*task.get_agents()}):    # Agents in KnowledgeBase but not specified in Task-Goal
            #     print(agent)
            #     task.update_duration(agent, tasks_stasts_dict[task.get_type()][agent]["exp_duration"])

    def get_combinations(self) -> Dict[Tuple[str, str], str]:
        combinations = {}
        for task in self.task_list:
            for agent in task.get_agents():
                combinations[(agent, task.get_id())] = task.get_duration(agent)
        return combinations

    def get_tasks_list(self) -> List[str]:
        return [task.get_id() for task in self.task_list]

    def get_agents(self):
        return self.agents

    def get_nominal_upper_bound(self) -> float:
        return sum([task.get_max_duration() for task in self.task_list])

    def get_precedence_constraints(self):
        precedence_constraints = {}
        for task in self.task_list:
            precedence_constraints[task.get_id()] = task.get_precedence_constraints()
        return precedence_constraints

    def get_tasks_per_agent(self):
        tasks_per_agent = {}
        for task in self.task_list:
            for agent in task.get_agents():
                if agent not in tasks_per_agent:
                    tasks_per_agent[agent] = []
                tasks_per_agent[agent].append(task.get_id())
        return tasks_per_agent
