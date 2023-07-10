# from dataclasses import dataclass, field
# from typing import List, Dict, Optional
# import copy
# import rospy
# from std_msgs.msg import String, Bool
#
# from task_planner_interface_msgs.msg import MotionTaskExecutionRequest
# from Task import TaskSolution
# import threading
# import multiprocessing
#
# THREASHOLD = 20
#
# @dataclass
# class TaskDispatcher:
#     agent: str
#     request_publisher: rospy.Publisher = field(init=False)
#     feedback_subscriber: rospy.Subscriber = field(init=False)
#     busy: bool = field(default=False, init=False)
#     task_solution: List[TaskSolution] = field(init=False)
#     performed_tasks: List[TaskSolution] = field(default_factory=list,init=False) #TODO: TASK EXECUTION
#     failed: bool = field(default=False, init=False)
#     def __post_init__(self):
#         print(self.agent)
#         request_topic_name = "prova_request_"
#         feedback_topic_name = "prova_feedback_"
#
#         self.request_publishers = rospy.Publisher(request_topic_name + self.agent, String, queue_size=10)
#         self.feedback_subscribers = rospy.Subscriber(feedback_topic_name + self.agent, Bool,
#                                                      self.reset_agent)
#         # timeout = 100  # TODO Param
#         print(id(self.request_publishers))
#
#     def publish_request(self):
#         # print(id(self.request_publishers))
#         #
#         # print(type(self.task_solution))
#         dt = 0.1
#         rate = rospy.Rate(1.0 / dt)
#         # t = 0.0
#         # k = 0
#         t0 = rospy.Time.now()
#         # told = t
#         t_start = t0.to_sec()
#         while not rospy.is_shutdown():
#
#             if not self.task_solution:
#                 break
#
#             t = rospy.Time.now().to_sec() - t0.to_sec()
#
#             if t >= self.task_solution[0].get_start_time() and not self.busy:
#                 # print("dentroooooo")
#                 self.request_publishers.publish(String(self.task_solution[0].get_task().get_type()))
#                 # self.check_timeout()
#                 self.busy = True
#                 rospy.loginfo(
#                     f"Agent: {self.agent} Task: {self.task_solution[0].get_task().get_id()}, "
#                     f"t_start: {self.task_solution[0].get_start_time()}, "
#                     f"t_end: {self.task_solution[0].get_end_time()}")
#                 t_start = t
#                 # task_solution.remove(task_solution[0])
#             if t - t_start > THREASHOLD:
#                 self.failed = True
#                 print("Overtimeeeeee")
#                 break
#             # print(t)
#
#             # # <t += dt
#             # print(t-told -dt)
#             # told = t
#             rate.sleep()
#             # k+=1
#             # print(t)
#
#     def dispatch_solution(self, task_solutions: List[TaskSolution]):
#         task_solution = list(filter(lambda task_sol: task_sol.get_assignment() == self.agent, task_solutions))
#         task_solution.sort(key=lambda task_sol: task_sol.get_start_time())
#
#         self.task_solution = task_solution
#
#         thread = threading.Thread(target=self.publish_request)
#         thread.daemon = True
#         thread.start()
#
#     def get_flag(self):
#         return self.flag
#
#     def reset_agent(self, msg):
#         print(self.agent)
#         self.performed_tasks.append(self.task_solution[0])
#         self.task_solution.remove(self.task_solution[0])
#         self.busy = False
#
#     def get_performed_task(self):
#         if self.performed_tasks:
#             return self.performed_tasks.pop(0)
#         else:
#             return None

# TODO: FROM HERE OTHER SOLUTION


# from dataclasses import dataclass, field
# from typing import List, Dict, Optional
# import copy
# import rospy
# from std_msgs.msg import String, Bool
# import threading
# import multiprocessing
#
# from task_planner_interface_msgs.msg import MotionTaskExecutionRequest
# from Task import TaskSolution
#
# THREASHOLD = 10
# # def callback(data):
# #     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#
#
# @dataclass
# class TaskDispatcher:
#     agents: List[str]
#     request_publishers: Dict[str, rospy.Publisher] = field(default_factory=dict, init=False)
#     feedback_subscribers: Dict[str, rospy.Subscriber] = field(default_factory=dict, init=False)
#     busy: Dict[str, bool] = field(default_factory=dict, init=False)
#     # busy_time: Dict[str, float] = field(default_factory=dict, init=0)
#     task_solutions: Dict[str, List[TaskSolution]] = field(default_factory=dict, init=False)
#
#     def __post_init__(self):
#         self.busy = dict.fromkeys(self.agents, False)
#         # self.request_publishers = dict()
#         # self.feedback_subscribers = dict()
#         for agent in self.agents:
#             request_topic_name = "prova_request_"
#             feedback_topic_name = "prova_feedback_"
#
#             self.request_publishers[agent] = rospy.Publisher(request_topic_name + agent, String, queue_size=10)
#             self.feedback_subscribers[agent] = rospy.Subscriber(feedback_topic_name + agent, Bool,
#                                                                 copy.copy(self.reset_agent), agent)
#         # timeout = 100  # TODO
#
#     def publish_request(self, agent, task_solution: List[TaskSolution]):
#         # print(agent)
#         # for k in task_solution:
#         #     print(agent, k.get_task().get_id(), k.get_start_time())
#         # task_solution = list(task_solution)
#         # # print(task_solution)
#         # t = 0
#         # self.flag = False
#         # for task in task_solution:
#         #     # print(task)
#         #     print(f"Agent: {agent} perform: {task.get_task().get_id()}")
#         #     task_solution.remove(task)
#         #     rate.sleep()
#
#         # print("fdsfnsjfndsj")
#         dt = 0.1
#         rate = rospy.Rate(1.0 / dt)
#         # t = 0.0
#         # k = 0
#         t0 = rospy.Time.now()
#         # told = t
#         t_start = t0
#         while not rospy.is_shutdown():
#             if not task_solution:
#                 break
#
#             t = rospy.Time.now().to_sec() - t0.to_sec()
#             print(t)
#             print("gere")
#             # print(task_solution[0])
#             # print(task_solution[0].get_start_time())
#             print(len(task_solution))
#             print(self.busy[agent])
#             if t >= task_solution[0].get_start_time() and not self.busy[agent]:
#                 print("inside")
#                 print(len(task_solution))
#
#                 self.request_publishers[agent].publish(String(task_solution[0].get_task().get_type()))
#                 # self.check_timeout()
#                 self.busy[agent] = True
#                 rospy.loginfo(
#                     f"Agent: {agent} Task: {task_solution[0].get_task().get_id()}, t_start: {task_solution[0].get_start_time()}, t_end: {task_solution[0].get_end_time()}")
#                 task_solution.remove(task_solution[0])
#                 t_start = t
#             if t - t_start > THREASHOLD:
#                 print("Overtimeeeeee")
#                 break
#             # # <t += dt
#             # print(t-told -dt)
#             # told = t
#             rate.sleep()
#             # k+=1
#             # print(t)
#
#     def dispatch_solution(self, task_solutions: List[TaskSolution]):
#         # task_solutions: Dict[str, TaskSolution]
#         # task_solutions = {agent: list(filter(lambda task_solution: task_solution.get_assignment()
#         #                                                            == agent, task_solutions)) for agent in self.agents}
#         threads = list()
#         for agent in self.agents:
#             task_solution = list(filter(lambda task_sol: task_sol.get_assignment() == agent, task_solutions))
#             task_solution.sort(key=lambda task_sol: task_sol.get_start_time())
#             threads.append(threading.Thread(target=copy.copy(self.publish_request), args=(agent, task_solution),daemon=True))
#
#         for thread in threads:
#             thread.start()
#         import time
#         while not rospy.is_shutdown():
#             time.sleep(1)
#         if rospy.is_shutdown():
#             return
#         #
#         # print("*********************************************")
#         # print(task_solutions)
#         # for k in task_solutions:
#         #     print(k)
#         # for task in task_solutions[k]:
#         #     print(task.)
#         # print(task_solutions[k].get_task().get_id())
#         #
#
#         # \
#         #
#         #
#         #     [task_solution.get_assignment() for task_solution in task_solutions]
#         # thread.start()
#         # print("Ciao")
#
#     def get_flag(self):
#         return self.flag
#
#     def reset_agent(self, msg, agent):
#         print("resettato " + agent)
#         self.busy[agent] = False
#
#     # def increment_busy_time(self, agent, ):
#     #     self.busy_time[agent] +=
#
#     # def get_performed_tasks(self):


# ## TODO: LAST
from dataclasses import dataclass, field
from typing import List, Dict
import rospy
import threading

from task_planner_interface_msgs.msg import MotionTaskExecutionRequest, \
    MotionTaskExecutionRequestArray, \
    MotionTaskExecutionFeedback
from Task import TaskSolution
from utils import UserMessages

THREASHOLD = 100


@dataclass
class TaskDispatcher:
    agents: List[str]
    recipe_name: str = str()

    request_publishers: Dict[str, rospy.Publisher] = field(default_factory=dict, init=False)
    feedback_subscribers: Dict[str, rospy.Subscriber] = field(default_factory=dict, init=False)

    busy: Dict[str, bool] = field(default_factory=dict, init=False)
    busy_time: Dict[str, float] = field(default_factory=dict, init=False)

    task_solutions: Dict[str, List[TaskSolution]] = field(default_factory=dict, init=False)
    performed_tasks: List[TaskSolution] = field(default_factory=list, init=False)  # TODO: TASK EXECUTION
    task_number: Dict[str, float] = field(default_factory=dict, init=False)

    failed: bool = field(default=False, init=False)

    def __post_init__(self):
        self.busy = dict.fromkeys(self.agents, False)
        self.busy_time = dict.fromkeys(self.agents, 0)
        self.task_number = dict.fromkeys(self.agents, 0)

        # TODO: Add this param to launch file
        # if not rospy.has_param("/prefix"):
        #     rospy.logerr(UserMessages.PARAM_NOT_DEFINED_ERROR.value.format("prefix"))
        #     raise Exception
        # prefix_topic_name = rospy.get_param("/prefix") + "/"
        prefix_topic_name = '/sharework/test/stiima/'

        for agent in self.agents:
            # if not rospy.has_param(prefix_topic_name + agent + "/feedback"):
            #     rospy.logerr(UserMessages.PARAM_NOT_DEFINED_ERROR.value.format(prefix_topic_name + agent + "/feedback"))
            #     raise Exception()
            # if not rospy.has_param(prefix_topic_name + agent + "/request"):
            #     rospy.logerr(UserMessages.PARAM_NOT_DEFINED_ERROR.value.format(prefix_topic_name + agent + "/request"))
            #     raise Exception()
            if agent == "human_right_arm":
                agent_name = "human"
            else:
                agent_name = "motion"
            # request_publisher_topic_name = rospy.get_param(prefix_topic_name + agent + "/request")
            # feedback_subscriber_topic_name = rospy.get_param(prefix_topic_name + agent + "/feedback")
            request_publisher_topic_name = prefix_topic_name + agent_name + "/request"
            feedback_subscriber_topic_name = prefix_topic_name + agent_name + "/feedback"

            self.request_publishers[agent] = rospy.Publisher(request_publisher_topic_name,
                                                             MotionTaskExecutionRequestArray, queue_size=10)
            self.feedback_subscribers[agent] = rospy.Subscriber(feedback_subscriber_topic_name,
                                                                MotionTaskExecutionFeedback,
                                                                self.reset_agent, agent)

    def publish_task_request(self, agent: str, task_solution: TaskSolution):
        task = task_solution.get_task()
        task_request = MotionTaskExecutionRequest(task_id=task.get_type(),
                                                  task_name=task.get_id(),
                                                  expected_time=task.get_duration(agent),
                                                  t_start_planned=task_solution.get_start_time(),
                                                  t_end_planned=task_solution.get_end_time())
        request_array = MotionTaskExecutionRequestArray(cmd_id=self.task_number[agent],
                                                        tasks=[task_request])
        self.request_publishers[agent].publish(request_array)
        self.task_number[agent] += 1

    def publish_request(self):
        dt = 0.1
        rate = rospy.Rate(1.0 / dt)
        t0 = rospy.Time.now()
        self.busy_time = dict.fromkeys(self.agents, t0)
        while not rospy.is_shutdown():
            if any([len(task_list) == 0 for task_list in self.task_solutions.values()]):
                break
            for agent in self.agents:
                t = rospy.Time.now().to_sec() - t0.to_sec()
                agent_task_solution = self.task_solutions[agent]
                if not agent_task_solution:
                    break
                if t >= agent_task_solution[0].get_start_time() and not self.busy[agent]:
                    self.publish_task_request(agent, agent_task_solution[0])
                    rospy.loginfo(
                        UserMessages.DISPATCH_TASK_MSG.value.format(agent_task_solution[0].get_task().get_id(),
                                                                    agent))
                    # agent_task_solution[0].get_start_time(),

                    self.busy[agent] = True
                    self.busy_time[agent] = t

                # check if the feedback hasn't come in for a long time
                if t - self.busy_time[agent] > THREASHOLD:
                    self.failed = True
                    rospy.loginfo(UserMessages.TIMEOUT.value.format(agent))
                    break
            rate.sleep()

    def dispatch_solution(self, task_solutions: List[TaskSolution]):
        # task_solutions: Dict[str, TaskSolution]
        task_solutions.sort(key=lambda task_sol: task_sol.get_start_time())

        self.task_solutions = {agent: list(filter(lambda task_solution: task_solution.get_assignment()
                                                                        == agent, task_solutions)) for agent in
                               self.agents}
        thread = threading.Thread(target=self.publish_request)
        thread.daemon = True
        thread.start()

    def reset_agent(self, msg, agent):
        if self.task_solutions[agent]:
            if self.busy[agent]:
                task = self.task_solutions[agent][0].get_task()
                rospy.loginfo(UserMessages.FEEDBACK_TASK_MSG.value.format(task.get_type(), agent))
                self.performed_tasks.append(self.task_solutions[agent].pop(0))
            else:
                rospy.loginfo(UserMessages.FEEDBACK_WITHOUT_DISPATCH.value)
        else:
            rospy.loginfo(UserMessages.FEEDBACK_WITHOUT_DISPATCH.value)
        self.busy[agent] = False

    def is_failed(self):
        return self.failed

    def is_finished(self):
        if any([len(task_list) == 0 for task_list in self.task_solutions.values()]):
            return True
        return False

    def get_performed_task(self):
        if self.performed_tasks:
            return self.performed_tasks.pop(0)
        else:
            return None
