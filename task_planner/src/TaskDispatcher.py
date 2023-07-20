from dataclasses import dataclass, field
from typing import List, Dict
import rospy
import threading

from task_planner_interface_msgs.msg import MotionTaskExecutionRequest, \
    MotionTaskExecutionRequestArray, \
    MotionTaskExecutionFeedback
from Task import TaskSolution, TaskExecution
from utils import UserMessages
from std_msgs.msg import Float32, String

THREASHOLD = 150


@dataclass
class TaskDispatcher:
    agents: List[str]
    agents_group_name: Dict[str, str]

    recipe_name: str = str()

    request_publishers: Dict[str, rospy.Publisher] = field(default_factory=dict, init=False)
    feedback_subscribers: Dict[str, rospy.Subscriber] = field(default_factory=dict, init=False)

    busy: Dict[str, bool] = field(default_factory=dict, init=False)
    busy_time: Dict[str, float] = field(default_factory=dict, init=False)

    task_solutions: Dict[str, List[TaskSolution]] = field(default_factory=dict, init=False)
    performed_tasks: List[TaskSolution] = field(default_factory=list, init=False)  # TODO: TASK EXECUTION
    executed_tasks: List[TaskExecution] = field(default_factory=list, init=False)  #
    # t0: float = field(default_factory=0.0, init=False)  #

    task_number: Dict[str, int] = field(default_factory=dict, init=False)
    tot_task_number: Dict[str, int] = field(default_factory=dict, init=False)

    failed: bool = field(default=False, init=False)
    timeout: bool = field(default=False, init=False)

    completion_percentage: Dict[str, float] = field(default_factory=dict, init=False)
    task_number_publisher: Dict[str, rospy.Publisher] = field(default_factory=dict, init=False)
    completion_publisher: Dict[str, rospy.Publisher] = field(default_factory=dict, init=False)
    current_task_publisher: Dict[str, rospy.Publisher] = field(default_factory=dict, init=False)

    def reset_agents_info(self):
        self.busy = dict.fromkeys(self.agents, False)
        self.busy_time = dict.fromkeys(self.agents, 0)
        self.task_number = dict.fromkeys(self.agents, 0)
        self.completion_percentage = dict.fromkeys(self.agents, 0)
        self.failed = False
        self.timeout = False

    def __post_init__(self):
        self.reset_agents_info()
        if not rospy.has_param("/prefix"):
            rospy.logerr(UserMessages.PARAM_NOT_DEFINED_ERROR.value.format("prefix"))
            raise Exception
        prefix_topic_name = rospy.get_param("/prefix") + "/"
        # prefix_topic_name = '/sharework/test/stiima/'

        for agent, agent_group_name in self.agents_group_name.items():
            # if not rospy.has_param(prefix_topic_name + agent + "/feedback"):
            #     rospy.logerr(UserMessages.PARAM_NOT_DEFINED_ERROR.value.format(prefix_topic_name + agent + "/feedback"))
            #     raise Exception()
            # if not rospy.has_param(prefix_topic_name + agent + "/request"):
            #     rospy.logerr(UserMessages.PARAM_NOT_DEFINED_ERROR.value.format(prefix_topic_name + agent + "/request"))
            #     raise Exception()

            # request_publisher_topic_name = rospy.get_param(prefix_topic_name + agent + "/request")
            # feedback_subscriber_topic_name = rospy.get_param(prefix_topic_name + agent + "/feedback")
            request_publisher_topic_name = prefix_topic_name + agent
            feedback_subscriber_topic_name = prefix_topic_name + agent + "/feedback"

            self.request_publishers[agent_group_name] = rospy.Publisher(request_publisher_topic_name,
                                                                        MotionTaskExecutionRequestArray, queue_size=10)
            self.feedback_subscribers[agent_group_name] = rospy.Subscriber(feedback_subscriber_topic_name,
                                                                           MotionTaskExecutionFeedback,
                                                                           self.reset_agent, agent_group_name)
            self.task_number_publisher[agent_group_name] = rospy.Publisher(f"task_number_info_{agent_group_name}",
                                                                           Float32,
                                                                           queue_size=10)
            self.completion_publisher[agent_group_name] = rospy.Publisher(f"completition_info_{agent_group_name}",
                                                                          Float32,
                                                                          queue_size=10)
            self.current_task_publisher[agent_group_name] = rospy.Publisher(f"current_task_{agent_group_name}",
                                                                            String,
                                                                            queue_size=10)
            rospy.sleep(5)

    def publish_task_request(self, agent: str, task_solution: TaskSolution):
        task = task_solution.get_task()
        task_request = MotionTaskExecutionRequest(task_id=task.get_type(),
                                                  task_name=task.get_id(),
                                                  # expected_time=task.get_duration(agent),
                                                  t_start_planned=task_solution.get_start_time(),
                                                  t_end_planned=task_solution.get_end_time(),
                                                  recipe_name=self.recipe_name)

        request_array = MotionTaskExecutionRequestArray(cmd_id=self.task_number[agent],
                                                        tasks=[task_request])
        self.request_publishers[agent].publish(request_array)
        self.task_number[agent] += 1

        self.current_task_publisher[agent].publish(String(f"Agent: {agent}, Task: {task.get_type()}"))
        self.task_number_publisher[agent].publish(float(self.task_number[agent]) / self.tot_task_number[agent] * 100.0)

    def publish_request(self):

        dt = 0.1
        rate = rospy.Rate(1.0 / dt)
        t0 = rospy.Time.now()
        self.t0 = t0.to_sec()
        self.busy_time = dict.fromkeys(self.agents, t0.to_sec())

        while not rospy.is_shutdown():
            if all([len(task_list) == 0 for task_list in self.task_solutions.values()]):
                print("Empty all")
                break
            for agent in self.agents:
                t = rospy.Time.now().to_sec() - self.t0
                agent_task_solution = self.task_solutions[agent]
                if not agent_task_solution:
                    # print("finito agente "+ agent)
                    continue
                if t >= agent_task_solution[0].get_start_time() and not self.busy[agent]: # and (not self.failed):
                    self.executed_tasks.append(TaskExecution(task=agent_task_solution[0].get_task(),
                                                             t_start=t,
                                                             assignment=agent))

                    print(f"Agente: {agent}, Discrepanza: {t - agent_task_solution[0].get_start_time()}")
                    self.publish_task_request(agent, agent_task_solution[0])
                    rospy.loginfo(
                        UserMessages.DISPATCH_TASK_MSG.value.format(agent_task_solution[0].get_task().get_id(), agent))
                    self.busy[agent] = True
                    # self.completion_percentage[agent] = 0
                    self.busy_time[agent] = t

                if self.busy[agent]:
                    if (agent_task_solution[0].get_end_time() - agent_task_solution[0].get_start_time()) > 0:

                        self.completion_percentage[agent] = (t - self.busy_time[agent]) / (
                                agent_task_solution[0].get_end_time() -
                                agent_task_solution[0].get_start_time()) * 100
                    else:
                        print("-------------------ERROREEEEEE ---------------")
                        print(agent_task_solution[0].get_task().get_id())
                        print(agent_task_solution[0].get_end_time())
                        print(agent_task_solution[0].get_start_time())
                        print(agent_task_solution[0])
                    self.completion_publisher[agent].publish(self.completion_percentage[agent])
                else:
                    self.completion_percentage[agent] = 0
                # Check if the feedback hasn't come in for a long time
                if t - self.busy_time[agent] > THREASHOLD:
                    self.timeout = True
                    # TODO: Da gestire non va bene gestirlo con failed perche' lista non vuota!
                    self.task_solutions[agent] = []
                    self.failed = True
                    rospy.loginfo(UserMessages.TIMEOUT.value.format(agent))
                    break
            rate.sleep()

    def dispatch_solution(self, task_solutions: List[TaskSolution]):
        self.reset_agents_info()
        # task_solutions: Dict[str, TaskSolution]
        task_solutions.sort(key=lambda task_sol: task_sol.get_start_time())

        self.task_solutions = {agent: list(filter(lambda task_solution: task_solution.get_assignment()
                                                                        == agent, task_solutions)) for agent in
                               self.agents}

        for agent in self.agents:
            self.tot_task_number[agent] = len(self.task_solutions[agent])
            print(f"Total task to perform by agent: {agent} is: {self.tot_task_number[agent]}")

        self.thread = threading.Thread(target=self.publish_request)
        self.thread.daemon = True
        self.thread.start()

    def reset_agent(self, msg: MotionTaskExecutionFeedback, agent: str):
        if self.task_solutions[agent]:
            if self.busy[agent]:
                task = self.task_solutions[agent][0].get_task()
                msg: MotionTaskExecutionFeedback
                outcome = msg.result
                if outcome == 0:
                    rospy.loginfo(UserMessages.FEEDBACK_FAILED_TASK_MSG.value.format(task.get_type(), agent))
                    self.failed = True
                else:
                    rospy.loginfo(UserMessages.FEEDBACK_TASK_MSG.value.format(task.get_type(), agent))
                finished_task = self.task_solutions[agent].pop(0)
                self.performed_tasks.append(finished_task)
                finished_task: TaskSolution
                tnow = rospy.Time.now()
                for task in self.executed_tasks:
                    if task.get_task().get_id() == finished_task.get_task().get_id():
                        pass
                        # print(task.get_task().get_id())
                        # print(finished_task.get_task().get_id())

                        # print("Stesso task")
                        # task.set_as_completed(rospy.Time.now().to_sec() - self.t0)
                print(rospy.Time.now().to_sec() - tnow.to_sec())
                self.completion_publisher[agent].publish(0)
            else:
                rospy.loginfo(UserMessages.FEEDBACK_WITHOUT_DISPATCH.value)
        else:
            rospy.loginfo(UserMessages.FEEDBACK_WITHOUT_DISPATCH.value)
        self.busy[agent] = False

    def is_failed(self):
        return self.failed

    def is_timeout(self):
        return self.timeout

    def is_failed_and_not_in_execution(self):
        print(f"Task da completare: {len(self.task_solutions)}, agenti impegnati: {self.busy}")
        # print([agent_busy for agent_busy in self.busy.items()])
        if all([len(task_list) == 0 for task_list in self.task_solutions.values()]):
            if self.is_failed() and all([not agent_busy for agent_busy in self.busy.values()]):
                return True
            else:
                return False
        else:
            return False

    def is_finished(self):
        if all([len(task_list) == 0 for task_list in self.task_solutions.values()]):
            # print([agent_busy for agent_busy in self.busy.items()])
            print([not (agent_busy) for agent_busy in self.busy.values()])

            if all([not agent_busy for agent_busy in self.busy.values()]):
                print("Plan finished")
                # print(self.executed_tasks)
                return True
        return False

    def get_performed_task(self):
        if self.performed_tasks:
            return self.performed_tasks.pop(0)
        else:
            return None

    def send_recipe_end(self):
        for agent in self.agents:
            task_request = MotionTaskExecutionRequest(task_id="end",
                                                      recipe_name=self.recipe_name)

            request_array = MotionTaskExecutionRequestArray(cmd_id=self.task_number[agent],
                                                            tasks=[task_request])
            self.request_publishers[agent].publish(request_array)
            self.task_number[agent] += 1

    def set_recipe_name(self, recipe_name: str):
        self.recipe_name = recipe_name
    # def go_home(self, agent):
    #     task_request = MotionTaskExecutionRequest(task_id="go_home",
    #                                               recipe_name=self.recipe_name)
    #
    #     request_array = MotionTaskExecutionRequestArray(cmd_id=100,
    #                                                     tasks=[task_request])
    #     self.request_publishers[agent].publish(request_array)
