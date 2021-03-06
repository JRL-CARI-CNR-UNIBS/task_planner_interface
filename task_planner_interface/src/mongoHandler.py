#! /usr/bin/env python3

import rospy

from std_srvs.srv import SetBool,SetBoolResponse
from std_msgs.msg import String

from task_planner_interface_msgs.msg import MotionTaskExecutionRequest, MotionTaskExecutionRequestArray, MotionTaskExecutionFeedback
from task_planner_interface_msgs.srv import TaskResult,TaskResultResponse,BasicSkill,BasicSkillResponse,TaskType,TaskTypeResponse,PickPlaceSkill,PickPlaceSkillResponse, PauseSkill, PauseSkillResponse  

import time

from pymongo import MongoClient
import pymongo.errors
from datetime import datetime

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'

PARAM_NOT_DEFINED_ERROR = "Parameter: {} not defined"
SERVICE_CALLBACK = GREEN + "Service call {} received" + END
CONNECTION_FAILED = "Connection to db failed: Request Timeout"
CONNECTION_OK = GREEN + "Connection to db executed" + END
READY = GREEN + "Ready to manage db" + END
UPDATE_OK = GREEN + "Update performed correctly" + END
SUCCESSFUL = "Successfully executed"
NOT_SUCCESSFUL = "Not Successfully executed"
CONNECTION_LOST = RED + "Connection to Database lost" + END
TASK_NO_PRESENT = RED + "Task properties not present"+ END
CHECK_OK = GREEN + "Check performed correctly" + END
PROPERTIES_MESSAGE = GREEN + "Properties retrieved correctly" + END

class MongoHandler:
    
    def __init__(self,db_name,coll_properties_name,coll_results_name,coll_duration_name,coll_risk_name):
        """Constructor

        Args:
            db_name (string): Database name
            coll_properties_name (string): Collection name with task properties
            coll_results_name (string): Collection name in which saving task results
            coll_duration_name (string): Collection name in which save "single-agents" task statistical information
            coll_risk_name (string): Collection name in which save elements of dynamic risk matrix

        Returns:
            string: Collection name
        """
        
        client = MongoClient(serverSelectionTimeoutMS=5000)     # 5 seconds of maximum connection wait
        try:
            client.server_info()
            rospy.loginfo(GREEN + CONNECTION_OK + END)
        except pymongo.errors.ServerSelectionTimeoutError:
            rospy.logerr(RED + CONNECTION_FAILED + END)
            raise
        pass 
        self.db = client[db_name]
        self.coll_skills = self.db[coll_properties_name]
        self.coll_results = self.db[coll_results_name]
        self.coll_interaction = self.db[coll_risk_name]
        self.coll_durations = self.db[coll_duration_name]

    
    def checkTaskType(self,task_type_msg):
        """Service callback for check if a task name exist end return task type

        Args:
            task_type_msg (TaskType): message containing task name
        Returns:
            TaskTypeResponse: message containing task type and an attribute error.
        """
        rospy.loginfo(SERVICE_CALLBACK.format("for check type"))
        
        document = {
            "name" : task_type_msg.name,
        }
        project = {
            "_id" : 0,
            "type" : 1
        }
        
        task_type = TaskTypeResponse()
        
        try:
            query_result = self.coll_skills.find_one(document,project) 
            
            if query_result == None:
                task_type.exist = False
                rospy.logerr(TASK_NO_PRESENT)
            else:
                task_type.exist = True
                task_type.type = query_result["type"]
                rospy.loginfo(CHECK_OK)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            task_type.exist = False
        return task_type
        
    def uploadResult(self,task_result_msg):
        """Service callback for update single task results

        Args:
            task_result_msg (TaskResult): Message containing task results informations

        Returns:
            [TaskResultResponse]: Message containing a boolean true if succesfully executed or false otherwhise.

        """
        
        rospy.loginfo(SERVICE_CALLBACK.format("for update result"))

        document = {
            "name" : task_result_msg.name,
            "type" : task_result_msg.type,
            "outcome" : task_result_msg.outcome,
            "duration_planned" : task_result_msg.duration_planned,
            "duration_real" : task_result_msg.duration_real,
            "planning_time" : task_result_msg.planning_time,
            "path_length" : task_result_msg.path_length,   
            "date" : datetime.utcnow(),
            "recipe" : task_result_msg.recipe,
            "agent" : task_result_msg.agent,
            "t_start": task_result_msg.t_start.to_sec(),
            "t_end": task_result_msg.t_end.to_sec()
        }
        
        try:
            self.coll_results.insert_one(document) 
            rospy.loginfo(UPDATE_OK)
            return TaskResultResponse(True,SUCCESSFUL)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return TaskResultResponse(False,NOT_SUCCESSFUL)
        
        
    def getTaskProperties(self,task_info):
        """Service callback for get a task properties

        Args:
            task_info (BasicSkill): BasicSkill msg with task name and type specification 

        Returns:
            BasicSkillResponse: BasicSkillResponse msg with task properties (query result)
        """
        
        rospy.loginfo(SERVICE_CALLBACK.format("for get task properties"))
        task_name = task_info.name
        query =  {
            "name" : task_name
        }
        
        skill_response = BasicSkillResponse()
        
        try:
            query_result = self.coll_skills.find_one(query)
        except pymongo.errors.AutoReconnect:                     # Db connection failed
            rospy.logerr(CONNECTION_LOST)
            skill_response.error = True
            return skill_response
        
        
        if query_result==None:                                  # Task properties not found
            rospy.logerr(TASK_NO_PRESENT)
            skill_response.error = True
        else:
            rospy.loginfo(PROPERTIES_MESSAGE)
            skill_response.error = False
            skill_response.name = query_result["name"]
            skill_response.type = query_result["type"]
            skill_response.description = query_result["description"]
            skill_response.agent = query_result["agent"]
            skill_response.goal = query_result["goal"]
            if "job_name" in query_result.keys():
                skill_response.job_name = query_result["job_name"]                      
        return skill_response

    def getPickPlaceProperties(self,task_info):    
        """Service callback for get a task properties (pickplace)

        Args:
            task_info (PickPlaceSkill): PickPlaceSkill msg with task name and type specification 

        Returns:
            PickPlaceSkillResponse: PickPlaceSkillResponse msg with task properties (query result)
        """
        
        rospy.loginfo(SERVICE_CALLBACK.format("for get human task properties"))
        
        task_name = task_info.name
        query =  {
            "name" : task_name
        }
        
        skill_response = PickPlaceSkillResponse()
        
        try:
            query_result = self.coll_skills.find_one(query)
        except pymongo.errors.AutoReconnect:                     # Db connection failed
            rospy.logerr(CONNECTION_LOST)
            skill_response.error = True
            return skill_response
        
        
        if query_result==None:                                  # Task properties not found
            rospy.logerr(TASK_NO_PRESENT)
            skill_response.error = True
        else:
            rospy.loginfo(PROPERTIES_MESSAGE)
            skill_response.error = False
            skill_response.name = query_result["name"]
            skill_response.type = query_result["type"]
            skill_response.description = query_result["description"]
            skill_response.agent = query_result["agent"]
            skill_response.pick_goal = query_result["pick_goal"]
            skill_response.place_goal = query_result["place_goal"]            
            if "job_name" in query_result.keys():
                skill_response.job_name = query_result["job_name"]                        

        return skill_response  
        
    def getPauseProperties(self,task_info):
        """Service callback for get a task properties for pause task

        Args:
            task_info (PauseSkill): PauseSkill msg with task name and type specification 

        Returns:
            PauseSkillResponse: PauseSkillResponse msg with task properties (query result)
        """
        
        rospy.loginfo(SERVICE_CALLBACK.format("for get pause properties"))
        task_name = task_info.name
        query =  {
            "name" : task_name
        }
        
        skill_response = PauseSkillResponse()
        
        try:
            query_result = self.coll_skills.find_one(query)
        except pymongo.errors.AutoReconnect:                     # Db connection failed
            rospy.logerr(CONNECTION_LOST)
            skill_response.error = True
            return skill_response
        
        if query_result==None:                                  # Task properties not found
            rospy.logerr(TASK_NO_PRESENT)
            skill_response.error = True
        else:
            rospy.loginfo(PROPERTIES_MESSAGE)
            skill_response.error = False
            skill_response.name = query_result["name"]
            skill_response.type = query_result["type"]
            skill_response.idle_time = query_result["idle_time"]                      
        return skill_response   
    
    def getHumanTaskProperties(self,task_info):
        rospy.loginfo(SERVICE_CALLBACK.format("for get task properties"))
        task_name = task_info.name
        query =  {
            "name" : task_name
        }
        
        skill_response = BasicSkillResponse()
        
        try:
            query_result = self.coll_skills.find_one(query)
        except pymongo.errors.AutoReconnect:                     # Db connection failed
            rospy.logerr(CONNECTION_LOST)
            skill_response.error = True
            return skill_response
        
        
        if query_result==None:                                  # Task properties not found
            rospy.logerr(TASK_NO_PRESENT)
            skill_response.error = True
        else:
            rospy.loginfo(PROPERTIES_MESSAGE)
            skill_response.error = False
            skill_response.name = query_result["name"]
            skill_response.type = query_result["type"]
            skill_response.description = query_result["description"]
            skill_response.agent = query_result["agent"]                  
        return skill_response        
   
def main():

    rospy.init_node("mongo_handler")
    
    try:
        db_name=rospy.get_param("~mongo_database")   
    except KeyError:   
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("mongo_database") + END)
        return 0
    try:
        coll_properties_name=rospy.get_param("~mongo_collection_tasks")   
    except KeyError:   
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("mongo_collection_tasks") + END)
        return 0
    try:
        coll_results_name=rospy.get_param("~mongo_collection_results")   
    except KeyError:   
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("mongo_collection_results") + END)
        return 0
    try:
        coll_duration_name=rospy.get_param("~coll_duration_name")   
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("coll_duration_name") + END)
        return 0
    try:
        coll_risk_name=rospy.get_param("~coll_risk_name")   
    except KeyError:        
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("coll_risk_name") + END)
        return 0    

    try:
        mongo_handler = MongoHandler(db_name,coll_properties_name,coll_results_name,coll_duration_name,coll_risk_name)
    except:
        return 0     #Connection to db failed 
    
       
    # Rosservice
    rospy.Service("mongo_handler/task_result",TaskResult,mongo_handler.uploadResult)
    rospy.Service("mongo_handler/get_task_properties",BasicSkill,mongo_handler.getTaskProperties)
    rospy.Service("mongo_handler/get_task_properties_pickplace",BasicSkill,mongo_handler.getPickPlaceProperties)
    rospy.Service("mongo_handler/get_pause_properties",PauseSkill,mongo_handler.getPauseProperties)
    
    rospy.Service("mongo_handler/get_task_properties_human",BasicSkill,mongo_handler.getHumanTaskProperties)

    rospy.Service("mongo_handler/check_task_type",TaskType,mongo_handler.checkTaskType)
    
    rospy.loginfo(READY)
    rospy.spin()

if __name__ == "__main__":
    main()
