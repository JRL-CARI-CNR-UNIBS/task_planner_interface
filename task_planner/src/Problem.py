from Task import Task, TaskSolution
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
import rospy
from task_planner_interface_msgs.srv import TaskType, TaskTypeResponse, TasksInformation, TasksInformationResponse, \
    TasksStatistics, TasksStatisticsResponse
from task_planner_statistics.srv import TaskSynergies, TaskSynergiesRequest, TaskSynergiesResponse
from utils import *
import itertools


@dataclass
class Problem:
    task_list: List[Task] = field(default_factory=list, init=False)
    agents: List[str]
    robot_agents: List[str] = field(default_factory=list, init=True)
    solution: List[TaskSolution] = field(default_factory=list, init=False)

    def __post_init__(self):
        rospy.wait_for_service('mongo_handler/check_task_type')
        rospy.wait_for_service('mongo_handler/get_task_agents')
        rospy.wait_for_service('mongo_handler/get_tasks_statistics')
        rospy.wait_for_service('mongo_statistics/get_task_synergies')

        self.task_check_srv = rospy.ServiceProxy("mongo_handler/check_task_type", TaskType)
        self.get_tasks_info_srv = rospy.ServiceProxy("mongo_handler/get_task_agents", TasksInformation)
        self.get_tasks_stats_srv = rospy.ServiceProxy("mongo_handler/get_tasks_statistics", TasksStatistics)
        self.get_task_synergies_srv = rospy.ServiceProxy("mongo_statistics/get_task_synergies", TaskSynergies)
        if self.robot_agents:
            if not self.__are_robots_agents_in_agents():
                raise ValueError("Some robots or agents are not present in the list of agents.")

    def add_task(self, task: Task) -> None:
        if task not in self.task_list:
            self.task_list.append(task)
        else:
            raise Exception("Task already present")

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
                    rospy.logerr(f"The precedence task: {precedence_task}, of task: {task.get_id()}, does not exist.")
                    return False

            for soft_precedence_task in task.get_soft_constraints():
                # Check if exist all soft precedence constraints among task_ids
                if soft_precedence_task not in tasks_dict.keys():
                    rospy.logerr(
                        f"The soft precedence task: {soft_precedence_task}, of task: {task.get_id()}, does not exist.")
                    return False

            # Check if all task type exist
            try:
                task_check_result = self.task_check_srv(task.get_type())
                if not task_check_result.exist:
                    rospy.logerr(
                        f"{Color.RED.value} Type {task.get_type()} not present in Knowledge Base {Color.END.value}")
                    return False
            except rospy.ServiceException as e:
                rospy.logerr(UserMessages.SERVICE_FAILED.value.format("check_task_type"))
                return False
        return True

    def fill_task_agents(self) -> bool:
        try:
            tasks_info_srv_result = self.get_tasks_info_srv()
        except rospy.ServiceException as e:
            rospy.logerr(UserMessages.SERVICE_FAILED.value.format("get_task_agents"))
            return False

        # Retrieve agents-task ability
        # task_agents_correspondence = {task_info.name: task_info.agents for task_info in tasks_info_result.tasks_info}
        task_agents_correspondence = {}
        for task_info in tasks_info_srv_result.tasks_info:
            task_agents_correspondence[task_info.name] = task_info.agents

        if not task_agents_correspondence:
            print(f"Task info empty from service: check DB!")
            return False

        for task in self.task_list:
            if not task.get_agents():  # Empty => Take mongoDB agents
                if task.get_type() not in task_agents_correspondence:
                    print(f"Task: {task.get_type()} not present in agents tasks ability from Knowledge Based")
                    return False
                task.update_agents(task_agents_correspondence[task.get_type()])

            # Check if the specified agent (in task goal) can actually perform it if not all(required_agent in
            # task_agents_correspondence[task.get_type()] for required_agent in task.get_agents()):
            for required_agent in task.get_agents_constraint():
                # Check if that agent exists at Knowledge-Based
                if required_agent not in self.agents:
                    rospy.loginfo(
                        f"{Color.RED.value}The specified agent: {required_agent}, for task: {task.get_id()},"
                        f" does not exist {Color.END.value}")
                    return False
                # Check if the specified agent (in task goal) can actually perform it
                if required_agent not in task_agents_correspondence[task.get_type()]:
                    rospy.loginfo(
                        f"{Color.RED.value}The specified agent: {required_agent}, "
                        f"for task: {task.get_id()}, cannot perform task: {task.get_id()} {Color.END.value}")
                    return False
        return True

    def update_tasks_statistics(self) -> bool:
        try:
            tasks_stats_result = self.get_tasks_stats_srv()
        except rospy.ServiceException as e:
            rospy.logerr(UserMessages.SERVICE_FAILED.value.format("get_tasks_statistics"))
            return False
        # utils: tasks_stasts_dict : {task_name: {agent1: {stats}, agent2: {stats}}}
        if not tasks_stats_result.statistics:
            print("Tasks statistics empty from service. Check DB")
            return False
        tasks_stasts_dict = {}
        for tasks_stats in tasks_stats_result.statistics:
            if tasks_stats.name not in tasks_stasts_dict:
                tasks_stasts_dict[tasks_stats.name] = {}
            if tasks_stats.agent not in tasks_stasts_dict[tasks_stats.name]:
                tasks_stasts_dict[tasks_stats.name][tasks_stats.agent] = {}

            tasks_stasts_dict[tasks_stats.name][tasks_stats.agent] = {"exp_duration": tasks_stats.exp_duration,
                                                                      "duration_std": tasks_stats.duration_std}
        # TODO: Pass in Task also the duration std
        for task in self.task_list:
            if task.get_type() not in tasks_stasts_dict.keys():
                print(f"Missing task statistics for: {task}")
                return False
            for agent in task.get_agents():
                if agent not in tasks_stasts_dict[task.get_type()].keys():
                    print(f"Missing task statistics for: {task}, for agent: {agent}")
                    return False
                task.update_duration(agent, tasks_stasts_dict[task.get_type()][agent]["exp_duration"])
                # TODO: In future will be usefull store also duration_std
            # Update also statistics for agents present in Knowledge base but not in task-Goal
            # for agent in set(tasks_stasts_dict[task.get_type()].keys()).difference({*task.get_agents()}):    # Agents in KnowledgeBase but not specified in Task-Goal
            #     task.update_duration(agent, tasks_stasts_dict[task.get_type()][agent]["exp_duration"])
        return True

    def update_tasks_synergy(self) -> bool:
        if not self.task_list:
            print("Empty task list")
            return False
        for task in self.task_list:
            print(task)
            task_type = task.get_type()
            task_agents = task.get_agents()
            if (not task_type) or (not task_agents):
                print(f"Task type or agents for task: {task} not present in Problem")
                return False
            for agent in task_agents:
                task_synergies_request = TaskSynergiesRequest()
                task_synergies_request.task_name = task_type
                task_synergies_request.agent = agent
                try:
                    task_synergies_result = self.get_task_synergies_srv(task_synergies_request)
                except rospy.ServiceException as e:
                    rospy.logerr(UserMessages.SERVICE_FAILED.value.format("get_task_synergies"))
                    return False
                synergies = task_synergies_result.synergies
                if not synergies:
                    print(f"Synergies for task: {task_type}, agent: {agent} empty.")
                    return False
                synergies_dict = dict()
                for task_synergy in synergies:
                    assert task_synergy.agent is not agent
                    if task_synergy.agent not in synergies_dict:
                        synergies_dict[task_synergy.agent] = dict()
                    synergies_dict[task_synergy.agent][task_synergy.task_name] = task_synergy.synergy
                for parallel_agent in synergies_dict:
                    task.update_synergy(agent, parallel_agent, synergies_dict[parallel_agent])
        return True
        # TODO: Is better to store as an object with all the info. Can i recycle TaskSynergy message? But i cannot add method.
        # task_synergy.std_err
        # task_synergy.success_rate

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

    def get_soft_precedence_constraints(self):
        soft_precedence_constraints = {}
        for task in self.task_list:
            soft_precedence_constraints[task.get_id()] = task.get_soft_constraints()
        return soft_precedence_constraints

    def get_tasks_per_agent(self):
        tasks_per_agent = {}
        for task in self.task_list:
            for agent in task.get_agents():
                if agent not in tasks_per_agent:
                    tasks_per_agent[agent] = []
                tasks_per_agent[agent].append(task.get_id())
        return tasks_per_agent

    def get_not_enabled_agents_constraints(self):
        not_enabled_agents = {}
        for task in self.task_list:
            not_enabled_agents[task.get_id()] = task.get_not_enabled_agents()
        return not_enabled_agents

    def get_robot_agents(self) -> List[str]:
        return self.robot_agents

    def get_not_robot_agents(self) -> List[str]:
        return list(set(self.agents).difference(set(self.robot_agents)))

    def __are_robots_agents_in_agents(self) -> bool:
        return set(self.robot_agents).issubset(set(self.agents))

    def add_task_solution(self, task_id: str, t_start: float, t_end: float, assignment: str) -> TaskSolution:
        # It can be unusefull have a solution in problem class
        for task in self.task_list:
            if task.get_id() == task_id:
                try:
                    task_solution = TaskSolution(task, t_start, t_end, assignment)
                except ValueError:
                    raise ValueError
                self.solution.append(task_solution)
                return task_solution
        raise Exception("Task not present in problem task list")

    def remove_task(self, task: Task):
        if task in self.task_list:
            self.task_list.remove(task)

    # def get_tasks_synergy(self):
    #     for task in self.task_list:
    #         task.get_synergies()
    #     pass
    # TODO: Finire

    def print_synergies(self):
        for task in self.task_list:
            print(task.get_agents())
            print(task.get_synergies())

    def get_tasks_synergies(self) -> Dict[Tuple[str, str, str, str], float]:
        tasks_synergies = {}
        tasks_type = []
        for task in self.task_list:
            task_type = task.get_type()
            # print(tasks_type)
            if task_type not in tasks_type:
                tasks_type.append(task_type)

                synergies = task.get_synergies()
                for agent, parallel_agent in synergies.keys():
                    for parallel_task in synergies[(agent, parallel_agent)]:
                        # TODO: Devo sapere come è formattata la synergy del task per farlo meglio una classe
                        tasks_synergies[(task_type, agent, parallel_task, parallel_agent)] = \
                            synergies[(agent, parallel_agent)][parallel_task]
        return tasks_synergies

    def get_tasks_type_correspondence(self) -> Dict[str, str]:
        tasks_type_correspondence = {}
        for task in self.task_list:
            tasks_type_correspondence[task.get_id()] = task.get_type()
        return tasks_type_correspondence

    def get_max_overlapping(self) -> float:
        tasks_durations = []
        for task in self.task_list:
            tasks_durations.append(task.get_max_duration())
        return max(tasks_durations)

    def get_near_tasks(self) -> List[Tuple[str, str]]:
        """
        Function that return the near tasks based on a yaml definition of tasks

        Args:
            neighboring_tasks:

        Returns:

        """
        # print(rospy.get_param_names())

        if not rospy.has_param("~neighboring_tasks_pairs"):
            raise ValueError(UserMessages.PARAM_NOT_DEFINED_ERROR.value.format("neighboring_tasks"))

        neighboring_tasks = rospy.get_param("~neighboring_tasks_pairs")
        print([len(task_pair) for task_pair in neighboring_tasks])
        if isinstance(neighboring_tasks, list) is not True:
            raise ValueError("neighboring_tasks must be a list")
        if all(len(task_pair) == 2 for task_pair in neighboring_tasks) is not True:
            raise ValueError("All pairs must contains two task")
        if all(self.task_in_task_list(task_1) and self.task_in_task_list(task_1)
               for task_1, task_2 in neighboring_tasks) is not True:
            raise ValueError("All elements in pairs must be in tasks list")
        neighboring_tasks = [(task_1, task_2) for task_1, task_2 in neighboring_tasks]

        return neighboring_tasks

    def task_in_task_list(self, task_id: str) -> bool:
        return any(task.get_id() == task_id for task in self.task_list)

    def get_robot_agents(self) -> Optional[List[str]]:
        return self.robot_agents

    def __repr__(self):
        if self.task_list:
            return_str = ""
            for task in self.task_list:
                print(task.__repr__())
                return_str += "\n"
                return_str += task.__repr__()
            return return_str
        else:
            return f"Problem instance. \nAgents: {self.agents}. \nEmpty Task List in Problem Definition"
