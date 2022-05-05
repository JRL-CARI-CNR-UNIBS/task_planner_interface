#ifndef human_node_01571456
#define human_node_01571456

#include <ros/ros.h>
#include <mongocpp_easy_driver/mongocpp_easy_driver.h>
#include <task_planner_interface/generic_skill.h>

#include <subscription_notifier/subscription_notifier.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequest.h>
#include <task_planner_interface_msgs/MotionTaskExecutionFeedback.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequestArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <bsoncxx/v_noabi/bsoncxx/document/view.hpp>

#define SUCCESS 1
#define FAILURE 0

void newCommandCallback(const task_planner_interface_msgs::MotionTaskExecutionRequestArrayConstPtr& msg){}
void newFeedbackCallback(const std_msgs::BoolConstPtr& msg){}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  /* Get params */
  std::string topic_request_from_planner;
  if (!pnh.getParam("topic_request_from_planner",topic_request_from_planner))
  {
    ROS_ERROR("topic_request_from_planner not defined");
    return false;
  }
  std::string topic_feedback_to_planner;
  if (!pnh.getParam("topic_feedback_to_planner",topic_feedback_to_planner))
  {
    ROS_ERROR("topic_feedback_to_planner not defined");
    return false;
  }
  std::string topic_feedback_from_hmi="task_feedback_from_hmi";
  if (!nh.getParam("topic_feedback_from_hmi",topic_feedback_from_hmi))
    ROS_ERROR("topic_feedback_from_hmi not defined. Deafult = 'task_feedback_from_hmi'");
  std::string group_name="human_real";
  if (!pnh.getParam("group_name",group_name))
    ROS_ERROR_STREAM(pnh.getNamespace() << ": group_name not defined. Default = 'human_real'");

  /* Subscribers and Publishers*/
  ros::Publisher fbk_to_planner_pub=nh.advertise<task_planner_interface_msgs::MotionTaskExecutionFeedback>(topic_feedback_to_planner,1);
  ros::Publisher cycletime_pub=nh.advertise<std_msgs::Float64>(pnh.getNamespace()+"/cycle_time",1);
  ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionRequestArray> command_notif(nh,topic_request_from_planner,1);
  ros_helper::SubscriptionNotifier<std_msgs::Bool> fbk_from_human_notif(nh,topic_feedback_from_hmi,1);
  command_notif.setAdvancedCallback(&newCommandCallback);
  fbk_from_human_notif.setAdvancedCallback(&newFeedbackCallback);

  std::string mongo_database="sharework3";
  if (!nh.getParam("mongo_database",mongo_database))
    ROS_ERROR_STREAM(nh.getNamespace() << ": mongo_database not defined. Default: " << mongo_database);
  else
    ROS_WARN_STREAM(nh.getNamespace() << ": mongo_database: " << mongo_database);

  std::string mongo_collection_tasks="hrc_task_properties";
  if (!nh.getParam("mongo_collection_tasks",mongo_collection_tasks))
    ROS_ERROR_STREAM(nh.getNamespace() << ": mongo_collection_tasks not defined. Default: " << mongo_collection_tasks);
  else
    ROS_WARN_STREAM(nh.getNamespace() << ": mongo_collection_results: " << mongo_collection_tasks);

  std::string mongo_collection_results="hrc_task_results";
  if (!nh.getParam("mongo_collection_results",mongo_collection_results))
    ROS_ERROR_STREAM(nh.getNamespace() << ": mongo_collection_results not defined. Default: " << mongo_collection_results);
  else
    ROS_WARN_STREAM(nh.getNamespace() << ": mongo_collection_results: " << mongo_collection_results);

  mongocpp_easy_driver::mongoEasyDriver mongo_handler(mongo_database);

  /* Ready to loop */
  task_planner_interface_msgs::MotionTaskExecutionRequestArray current_request_msg;
  task_planner_interface_msgs::MotionTaskExecutionRequest current_command;
  task_planner_interface_msgs::MotionTaskExecutionFeedbackPtr fbk_to_planner_msg(new task_planner_interface_msgs::MotionTaskExecutionFeedback());
  std_msgs::Bool fbk_from_human_msg;
  int current_result = FAILURE;
  bool is_dispatching = false;
  ros::Duration t_total;
  ros::Time t_start = ros::Time::now();
  ros::Rate r(100);

  while(ros::ok())
  {
    t_total=ros::Time::now()-t_start;
    ROS_INFO_THROTTLE(120,"Alive. Current cycle time = %f", t_total.toSec());
    if (!command_notif.waitForANewData(ros::Duration(60.0))){
      if (is_dispatching)
        ROS_INFO_STREAM( pnh.getNamespace() << ": 60 seconds without new commands.");
    }
    else
    {
      ROS_INFO_STREAM( pnh.getNamespace() << ": new command received!");
      if (!is_dispatching)
      {
        is_dispatching=true; // dispatching begins now
        t_start = ros::Time::now();
      }
      current_request_msg = command_notif.getData();
      current_command=current_request_msg.tasks.at(0);
      taskPlannerInterface::skills::GenericSkillPtr current_skill(new taskPlannerInterface::skills::GenericSkill());;

      std::string skill_type;
      bsoncxx::stdx::optional<bsoncxx::document::value> bson_doc;
      if (mongo_handler.findOneProjected(mongo_collection_tasks,"name",current_command.task_id,"type",skill_type))
      {
        mongo_handler.findOne(mongo_collection_tasks,"name",current_command.task_id,bson_doc);
        current_skill->taskPlannerInterface::skills::GenericSkill::setPropertiesFromBSON(bson_doc);
        /* Wait for feedback */
        ROS_INFO_STREAM( pnh.getNamespace() << ": Waiting for feedback...");
        if (!fbk_from_human_notif.waitForANewData(ros::Duration(60.0)))
        {
          //ROS_ERROR_STREAM(pnh.getNamespace() << ". timeout: no new messages from topic " << topic_feedback_from_hmi << ". return FAILURE.");
          //current_result = FAILURE;
        }
        else
        {
          fbk_from_human_msg=fbk_from_human_notif.getData();
          current_result = fbk_from_human_msg.data;
        }
        auto bson_results = current_skill->getResultsAsBSON();
        mongo_handler.appendToBSON("agent",group_name,bson_results);
        std::string recipe_name = "recipe_0";
        mongo_handler.appendToBSON("recipe",recipe_name,bson_results);
        std::vector<std::string> concurrent_tasks=current_request_msg.tasks.at(0).human_tasks;
        mongo_handler.appendArrayToBSON("concurrent_tasks",concurrent_tasks,bson_results);

        mongo_handler.insertOne(mongo_collection_results,bson_results);

      }
      else if (!current_command.task_id.compare("end")){
        current_result = SUCCESS;
        t_total=ros::Time::now()-t_start;
        std_msgs::Float64Ptr totaltime_msg(new std_msgs::Float64());
        totaltime_msg->data=t_total.toSec();
        cycletime_pub.publish(totaltime_msg);
        is_dispatching=false;
        ROS_WARN_STREAM(pnh.getNamespace() << ": plan completed in "<< t_total.toSec() <<" seconds. Waiting for a new recipe.");
      }
      else{
        ROS_FATAL_STREAM("Command" << current_command.task_id << "not defined. Abort plan.");
        break;
      }

      fbk_to_planner_msg->cmd_id = current_request_msg.cmd_id;
      fbk_to_planner_msg->result = current_result;
      fbk_to_planner_pub.publish(fbk_to_planner_msg);
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

#endif

