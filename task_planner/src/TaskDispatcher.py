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
                        UserMessages.DISPATCH_TASK_MSG.value.format(agent_task_solution[0].get_task().get_id(), agent))

                    self.busy[agent] = True
                    self.busy_time[agent] = t

                # Check if the feedback hasn't come in for a long time
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
