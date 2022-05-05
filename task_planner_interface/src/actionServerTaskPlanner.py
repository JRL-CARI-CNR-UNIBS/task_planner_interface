#! /usr/bin/env python3

import rospy

import actionlib

import task_planner_interface_msgs.msg
from std_msgs.msg import Duration

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'

PARAM_NOT_DEFINED_ERROR = "Parameter: {} not defined"
BASE_TOPIC_NAME = "/sharework/test/stiima/"
ACTION_NAME = "/task_execute"

class TaskExecuteActions():
    # create messages that are used to publish feedback/result
    _feedback = task_planner_interface_msgs.msg.TaskExecuteFeedback()

    _result = task_planner_interface_msgs.msg.TaskExecuteResult()

    def __init__(self,agent):
        self.agent = agent
        rospy.loginfo("Action Server for agent: "+ self.agent + " Ok")
        self._as = actionlib.SimpleActionServer(BASE_TOPIC_NAME + self.agent + ACTION_NAME, task_planner_interface_msgs.msg.TaskExecuteAction, execute_cb=self.execute, auto_start = False)
        self._as.start()
        self.n=0

    def execute(self, goal):
        
        # helper variables
        r = rospy.Rate(1)
        success = True

        # append the seeds for the fibonacci sequence
        self._feedback.feedback = True

        # publish info to the console for the user
        rospy.loginfo("Executing " +self.agent+" action")
        # start executing the action

        rospy.loginfo("Task name: " + goal.name)
        print(self.n)
        if self._as.is_preempt_requested():
            rospy.loginfo("Action cancellata")
            self._as.set_preempted()
            success = False

            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
        for k in range(0,5):
            print(k)
            r.sleep()
        if success:

            self._result.outcome = self.n
            self._result.type = "lo deve passare l'executor"
            # self._result.duration_planned = Duration()
            # self._result.duration_planned.data.secs=1
            # print()
            # self._result.duration_real = Duration()
            # self._result.duration_real.data.secs=1
            # self._result.planning_time = Duration()
            # self._result.planning_time.data.secs=1
            rospy.loginfo("Eseguito con successo")
            self.n+=1
            self._as.set_succeeded(self._result)
        


def main():
    rospy.init_node('task_executor_action_server',anonymous=True)
    try:
        agent=rospy.get_param("~agent")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("agent") + END)
        return 0
    server = TaskExecuteActions(agent)
    rospy.spin()
    
if __name__ == '__main__':
    main()

