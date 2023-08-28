from dataclasses import dataclass, field

import rospy
from std_msgs.msg import String, Bool
from task_planner_interface_msgs.msg import MotionTaskExecutionRequest, \
    MotionTaskExecutionRequestArray, \
    MotionTaskExecutionFeedback


@dataclass
class GuiInterface:
    gui_request_topic_name: str
    gui_feedback_topic_name: str
    task_name: str = field(init=False)
    gui_request_pub: rospy.Publisher = field(init=False)

    # gui_feedback_sub: rospy.Subscriber()= field(default_factory=None, init=False)

    def __post_init__(self):
        self.gui_request_pub = rospy.Publisher(self.gui_request_topic_name, String, queue_size=10)

    def request(self, msg):
        self.task_name = msg.tasks[0].task_id
        task_description = msg.tasks[0].task_description
        rospy.loginfo(f"Task request: {self.task_name}")

        msg_to_pub = String(self.task_name)

        self.gui_request_pub.publish(msg_to_pub)

        self.wait_response()

    def wait_response(self):
        received = False
        while not received:
            response = rospy.wait_for_message(self.gui_feedback_topic_name,
                                          Bool, 1000)
            if response.data:
                received = True
        rospy.loginfo(f"Received feedback for: {self.task_name}")
        feedback_msg = MotionTaskExecutionFeedback()
        feedback_msg.result = 1


def main():
    gui_request_topic_name = "prova_request"
    gui_feedback_topic_name = "prova_feedback"
    high_level_request_topic_name = "prova_high_level"

    rospy.init_node("gui_interface")

    gui_interface = GuiInterface(gui_request_topic_name, gui_feedback_topic_name)

    request_from_high_level = rospy.Subscriber(high_level_request_topic_name,
                                               MotionTaskExecutionRequestArray,
                                               gui_interface.request)

    rospy.spin()


if __name__ == "__main__":
    main()
