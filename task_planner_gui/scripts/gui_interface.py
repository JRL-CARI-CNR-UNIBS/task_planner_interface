#!/usr/bin/env python3

from dataclasses import dataclass, field
from typing import List

import rospy
from std_msgs.msg import String, Bool
from task_planner_interface_msgs.msg import MotionTaskExecutionRequest, \
    MotionTaskExecutionRequestArray, \
    MotionTaskExecutionFeedback


@dataclass
class GuiInterface:
    gui_request_topic_name: str
    gui_feedback_topic_name: str
    high_level_feedback_topic_name: str

    task_name: str = field(init=False)
    gui_request_pub: rospy.Publisher = field(init=False)
    n_task: int = field(default=0, init=False)
    performed_task: List[str] = field(default_factory=list, init=False)

    # gui_feedback_sub: rospy.Subscriber()= field(default_factory=None, init=False)

    def __post_init__(self):
        self.gui_request_pub = rospy.Publisher(self.gui_request_topic_name, String, queue_size=10)
        self.high_level_feedback_pub = rospy.Publisher(self.high_level_feedback_topic_name, MotionTaskExecutionFeedback,
                                                       queue_size=10)
        rospy.sleep(2)
        self.gui_request_pub.publish(String("Let's start"))

    def request(self, msg):
        self.task_name = msg.tasks[0].task_id
        task_description = msg.tasks[0].task_description
        rospy.loginfo(f"Task request: {self.task_name}")

        str_to_pub = ""
        # for task in self.performed_task:
        #     str_to_pub += f"PERFORMED: {task} \n"

        msg_to_pub = String(f"TODO: {self.task_name}... \n{str_to_pub}")
        self.gui_request_pub.publish(msg_to_pub)

        self.wait_response()

        self.performed_task.append(self.task_name)

    def wait_response(self):
        received = False
        while not received:
            response = rospy.wait_for_message(self.gui_feedback_topic_name, Bool, 1000)
            if response.data:
                received = True
        rospy.loginfo(f"Received feedback for: {self.task_name}")
        feedback_msg = MotionTaskExecutionFeedback()
        feedback_msg.result = 1
        self.high_level_feedback_pub.publish(feedback_msg)


def main():
    rospy.init_node("gui_interface")

    params = ["gui_request_topic_name",
              "gui_feedback_topic_name",
              "high_level_request_topic_name",
              "high_level_feedback_topic_name"]
    if all([rospy.has_param(param) for param in params]):
        gui_request_topic_name = rospy.get_param("gui_request_topic_name")
        gui_feedback_topic_name = rospy.get_param("gui_feedback_topic_name")

        high_level_request_topic_name = rospy.get_param("high_level_request_topic_name")
        high_level_feedback_topic_name = rospy.get_param("high_level_feedback_topic_name")

    else:
        print(([rospy.has_param(param) for param in params]))
        rospy.loginfo("Not all necessary params defined")
        return 0

    gui_interface = GuiInterface(gui_request_topic_name, gui_feedback_topic_name, high_level_feedback_topic_name)

    request_from_high_level = rospy.Subscriber(high_level_request_topic_name,
                                               MotionTaskExecutionRequestArray,
                                               gui_interface.request)

    rospy.spin()


if __name__ == "__main__":
    main()
