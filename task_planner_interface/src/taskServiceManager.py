#! /usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalID

from datetime import datetime

from std_srvs.srv import SetBool,SetBoolResponse
from std_msgs.msg import String, Duration, Float64

from task_planner_interface_msgs.msg import MotionTaskExecutionRequest, MotionTaskExecutionRequestArray, MotionTaskExecutionFeedback
from task_planner_interface_msgs.srv import TaskResult,TaskResultRequest, TaskResultResponse,BasicSkill,BasicSkillResponse,TaskType,TaskTypeResponse,PauseSkill,PauseSkillResponse
from task_planner_interface_msgs.msg import TaskExecuteAction, TaskExecuteGoal

import time
from datetime import datetime

from pymongo import MongoClient
import pymongo.errors

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'

# User messages
PARAM_NOT_DEFINED_ERROR = "Parameter: {} not defined"
TASK_NO_PRESENT = RED + "Task properties not present"+ END
MESSAGE_TASK_RECEIVED =  GREEN + "Task received : {}, from agent: {}" + END
SERVICE_FAILED = RED + "Service call failed : {}" + END
TASK_TYPE = "Task type: {}"
ACTION_CLIENT_MESSAGE = GREEN + "Added Action Client for: {}" + END
INPUT_MESSAGE = "Press Enter to continue..."
DATE_FORMAT = "%Y_%m_%d_%H:%M:%S"
START_PAUSE_MESSAGE = "Starting pause of: {} s"
FINISH_PAUSE_MESSAGE = GREEN + "Pause finished, for agent: {}" + END
END_MESSAGE = GREEN + "Recipe number: {} succesfully performed, cycletime: {}" + END
DATABASE_DISCONNECTED = RED + "Disconnected from database" + END

RECIPE_NAME_FORMAT = "{}_{}_{}"

#Topics and Services
UPLOAD_RESULT_SERVICE = "mongo_handler/task_result"
CHECK_TASK_TYPE_SERVICE = 'mongo_handler/check_task_type'
PAUSE_PROPERTIES = "mongo_handler/get_pause_properties"

ACTION_NAME = "/task_execute"
ACTION_NAME_SEQ = "/sequence_execute"

CYCLE_TIME_TOPIC_NAME = "/cycle_time"

class TaskServiceManager:

    def __init__(self,agent,agent_type,data,recipe_name,prefix_topic_name,is_human_real,start_recipe_index):
        """Constructor

        Args:
            agent (String): Name of agent
            agent_type (String): human or robot
            data (String): Actual data
            recipe_name (String): Recipe name
            prefix_topic_name (String): Prefix of topic name of action (executor) and feedback to upper layer
            is_human_real (bool): Flag for select if human agent is fake or real
            start_recipe_index (int): Starting recipe index
        """

        self.agent = agent                          # Agent
        self.agent_type = agent_type                # Agent type human or robot
        self.recipe_index = start_recipe_index      # Recipe Index
        self.data = data                            # Data
        self.recipe_name = recipe_name              # RecipeName
        self.cycle_initialized = False              # initialization (trigger for t_start)
        self.is_human_real = is_human_real          # Flag for select if human agent is fake or real

        # Publisher
        self.pub_feedback_task_planner = rospy.Publisher(prefix_topic_name + agent + "/feedback",MotionTaskExecutionFeedback,queue_size=10)
        self.pub_cycle_tyme = rospy.Publisher(CYCLE_TIME_TOPIC_NAME, Float64, queue_size=10)

        # Services Proxy
        self.task_type_service = rospy.ServiceProxy(CHECK_TASK_TYPE_SERVICE, TaskType)
        self.upload_result_service = rospy.ServiceProxy(UPLOAD_RESULT_SERVICE, TaskResult)
        self.pause_properties = rospy.ServiceProxy(PAUSE_PROPERTIES, PauseSkill)

        rospy.loginfo(YELLOW + "Waiting for services and action server..." + END)

        # Waiting for their existance
        rospy.wait_for_service(CHECK_TASK_TYPE_SERVICE)
        rospy.wait_for_service(UPLOAD_RESULT_SERVICE)
        rospy.wait_for_service(PAUSE_PROPERTIES)

        #Action Client

        self.action_client = actionlib.SimpleActionClient(prefix_topic_name + self.agent + ACTION_NAME,TaskExecuteAction)
        self.action_client.wait_for_server(rospy.Duration(5.0))
        self.cancel_pub = rospy.Publisher(prefix_topic_name + self.agent + ACTION_NAME+"/cancel", GoalID, queue_size=1)

        self.action_client_seq = actionlib.SimpleActionClient(prefix_topic_name + self.agent + ACTION_NAME_SEQ,TaskExecuteAction)
        self.action_client_seq.wait_for_server(rospy.Duration(5.0))
        self.cancel_pub = rospy.Publisher(prefix_topic_name + self.agent + ACTION_NAME_SEQ+"/cancel", GoalID, queue_size=1)

        rospy.loginfo(YELLOW + "Services and action server ready" + END)

        # Subscriber
        rospy.Subscriber(prefix_topic_name + agent, MotionTaskExecutionRequestArray, self.subscribeTask)      #Subscriber



    def subscribeTask(self,data):
        """Subscriber task from task planner

        Args:
            data ([MotionTaskExecutionRequestArray): Array of tasks request of execution
        """
        #rospy.loginfo(GREEN + "Task Received" + END)
        task = data.tasks[0].task_id
        task_cmd_id = data.cmd_id

        #rospy.loginfo(self.cycle_initialized)
        if not self.cycle_initialized:
            self.cycle_initialized = True
            self.t_start = rospy.Time.now()

        rospy.loginfo(MESSAGE_TASK_RECEIVED.format(task,self.agent))

        try:

            task_type_msg = self.task_type_service(task)            # Check type (rosservice call)

            if task_type_msg.exist:
                if self.is_human_real:
                    rospy.loginfo(TASK_TYPE.format(task_type_msg.type))
                    self.execute_client(task,task_cmd_id)
                else:
                    if task_type_msg.type in ["pick","place","pickplace","goto"] and task!="end":
                        rospy.loginfo(TASK_TYPE.format(task_type_msg.type))
                        # Call method for execute the task
                        self.execute_client(task,task_cmd_id)       # Il task name

                    elif task_type_msg.type in ["Assembly", "Disassembly", "Sequence", "unscrew", "Unscrew" ,"screw", "Screw"]:
                        rospy.loginfo(TASK_TYPE.format(task_type_msg.type))
                        # Call method for execute the sequence of sub-tasks
                        self.execute_sequence_client(task,task_cmd_id)       # Il task name

                    elif task == "end": #task e' name
                        #Note that in db task properties the end task has to be associated to all agents goal: it has to be ok for all agents. Home pose of all agent
                        self.execute_end(task,task_cmd_id)       # Il task name
                        # Publish cycle time and reset cycle_initialized
                        t_cycle = rospy.Time.now() - self.t_start
                        self.pub_cycle_tyme.publish(t_cycle.to_sec())
                        self.cycle_initialized = False

                        rospy.loginfo(END_MESSAGE.format(self.recipe_index,t_cycle.to_sec()))

                        self.recipe_index += 1  #Incement recipe index

                    elif task_type_msg.type == "pause":   #è = pause and exist in db properties

                        pause_properties_msg=self.pause_properties(task)    #task_name non task_type

                        rate = rospy.Rate(1.0/pause_properties_msg.idle_time)

                        rospy.loginfo(START_PAUSE_MESSAGE.format(pause_properties_msg.idle_time))
                        rate.sleep()
                        rospy.loginfo(FINISH_PAUSE_MESSAGE.format(self.agent))
                        pass
            elif task == "wait_for_input":
                input(INPUT_MESSAGE)
            else:       #Task doesn't exits
                rospy.loginfo(RED + "Goal not sent" + END)
                pass        #Da capire cosa fare nel caso non sia presente (Es. end, wait..oppure qualcosa che effettivamente non c'è)
        except rospy.ServiceException as e:
            rospy.logerr(SERVICE_FAILED.format(e))

    def execute_client(self,task_name,task_cmd_id):                   #task_name in realta' è il
        """Method for ask execution of task (calling action server)

        Args:
            task_name (String): Task name
            task_cmd_id (Int): cmd_id int identifier read by message usefull for feedback
        """
        #Check se gia in esec uno, cancella, aspetta, manda nuovo
        #rospy.loginfo(self.action_client.get_state())

        if self.action_client.get_state() !=  GoalStatus.SUCCEEDED and self.action_client.get_state()!=GoalStatus.LOST:
            self.action_client.cancel_all_goals()
            self.action_client.wait_for_result()
            rospy.loginfo(RED+"Previous goal deleted"+END)

        goal = TaskExecuteGoal(task_name)

        # tstart
        t_start_task = rospy.Time.now()
        self.action_client.send_goal(goal)
        rospy.loginfo(GREEN + "Goal Sent" + END)

        self.action_client.wait_for_result()
        action_result = self.action_client.get_result()
        # tend
        t_end_task = rospy.Time.now()
        rospy.loginfo(YELLOW + "Received result" + END)

        if self.action_client.get_state() ==  GoalStatus.SUCCEEDED:
            task_result = TaskResultRequest()
            task_result.name = task_name                # Name
            task_result.type = action_result.type
            task_result.agent = action_result.agent              # Agent
            task_result.outcome = action_result.outcome
            task_result.duration_planned = action_result.duration_planned
            task_result.duration_real = action_result.duration_real
            task_result.planning_time = action_result.planning_time
            task_result.path_length = action_result.path_length
            task_result.t_start = t_start_task
            task_result.t_end = t_end_task

            # Publish result to task planner
            self.pub_feedback_task_planner.publish(MotionTaskExecutionFeedback(task_cmd_id,action_result.outcome))

            #Define recipe name
            task_result.recipe = RECIPE_NAME_FORMAT.format(self.recipe_name,self.data,self.recipe_index)

            rospy.loginfo(task_result)

            result = self.upload_result_service(task_result)
            if result.success == False:
                #Cosa fare se mongo si e' disconnesso: il messaggio gia' in MongoHandler?
                pass

    def execute_sequence_client(self,task_name,task_cmd_id):                   #task_name in realta' è il
        """Method for ask execution of task (calling action server)

        Args:
            task_name (String): Task name
            task_cmd_id (Int): cmd_id int identifier read by message usefull for feedback
        """
        #Check se gia in esec uno, cancella, aspetta, manda nuovo
        #rospy.loginfo(self.action_client.get_state())

        if self.action_client_seq.get_state() !=  GoalStatus.SUCCEEDED and self.action_client_seq.get_state()!=GoalStatus.LOST:
            self.action_client_seq.cancel_all_goals()
            self.action_client_seq.wait_for_result()
            rospy.loginfo(RED+"Previous goal deleted"+END)

        goal = TaskExecuteGoal(task_name)
        # tstart
        t_start_task = rospy.Time.now()
        self.action_client_seq.send_goal(goal)
        rospy.loginfo(GREEN + "Goal Sent" + END)

        self.action_client_seq.wait_for_result()

        action_result = self.action_client_seq.get_result()
        # tend
        t_end_task = rospy.Time.now()

        rospy.loginfo(YELLOW + "Received result" + END)

        if self.action_client_seq.get_state() ==  GoalStatus.SUCCEEDED:
            task_result = TaskResultRequest()
            task_result.name = task_name                # Name
            task_result.type = action_result.type
            task_result.agent = self.agent              # Agent
            task_result.outcome = action_result.outcome
            task_result.duration_planned = action_result.duration_planned
            task_result.duration_real = action_result.duration_real
            task_result.planning_time = action_result.planning_time
            task_result.path_length = action_result.path_length
            task_result.t_start = t_start_task
            task_result.t_end = t_end_task

            # Publish result to task planner
            self.pub_feedback_task_planner.publish(MotionTaskExecutionFeedback(task_cmd_id,action_result.outcome))

            #Define recipe name
            task_result.recipe = RECIPE_NAME_FORMAT.format(self.recipe_name,self.data,self.recipe_index)

            rospy.loginfo(task_result)

            result = self.upload_result_service(task_result)
            if result.success == False:
                #Cosa fare se mongo si e' disconnesso: il messaggio gia' in MongoHandler?
                pass

    def execute_end(self,task_name,task_cmd_id):
        """Method for ask execution of task (calling action server)

        Args:
            task_name (String): Task name
            task_cmd_id (Int): cmd_id int identifier read by message usefull for feedback
        """
        goal = TaskExecuteGoal(task_name)
        self.action_client.send_goal(goal)

        self.action_client.wait_for_result()
        rospy.loginfo(GREEN + "End of recipe" + END)

def main():

    rospy.init_node("task_service_manager")

    #Required ROS parameters (agents)
    try:
        agents=rospy.get_param("/agents")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("agents") + END)
        return 0

    try:
        recipe_name=rospy.get_param("/recipe_name")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("recipe_name") + END)
        return 0
    try:
        prefix_topic_name=rospy.get_param("/prefix")
        prefix_topic_name=prefix_topic_name + "/"
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("prefix") + END)
        return 0

    try:
        is_human_real=rospy.get_param("/is_human_real")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("is_human_real") + END)
        return 0

    try:
        start_recipe_index=rospy.get_param("/starting_recipe_number")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("start_recipe_index") + END)
        start_recipe_index = 0
        rospy.logerr(RED + "Set to 0" + END)


    agent_task_manager = []     # List of object TaskServiceManager (one for each agent)
    agent_types = agents.keys()

    data = datetime.today().strftime(DATE_FORMAT)
    for agent_type in agent_types:
        for agent in agents[agent_type]:
            rospy.loginfo(prefix_topic_name + agent)
            if (agent_type=="human"):
                agent_task_manager.append(TaskServiceManager(agent,agent_type,data,recipe_name,prefix_topic_name,is_human_real,start_recipe_index))
            else:
                agent_task_manager.append(TaskServiceManager(agent,agent_type,data,recipe_name,prefix_topic_name,False,start_recipe_index))


    rospy.spin()

if __name__ == "__main__":
    main()
