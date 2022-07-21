
#include "reactive_task_planner_dispatcher/single_task_dispatcher.h"


SingleTaskDispatcher::SingleTaskDispatcher(const std::string &name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
{

  //   if(!FixtureCheck::getInput<std::string>("fixture_base_topic_name_",fixture_base_topic_name_))
  //   {
  //     ROS_ERROR("Missing required input [fixture_base_topic_name_]");
  //     throw BT::RuntimeError("Missing required input [fixture_base_topic_name_]");
  //   }

//  ROS_INFO_STREAM("Task name: " << task_name_);
//  fixture_state_sub_.reset(new ros_helper::SubscriptionNotifier<mqtt_scene_integration::Fixture> (nh_, "fixture_base_topic_name_"+task_name_,10));

}

void SingleTaskDispatcher::publishTask(const ros::Publisher &pub,
                                       const std::string &task_name)
{
    task_planner_interface_msgs::MotionTaskExecutionRequestArray task_to_publish;
    task_planner_interface_msgs::MotionTaskExecutionRequest task;
    task.task_id = task_name;
    //task_to_publish.cmd_id = 1;
    task_to_publish.tasks={task};
    pub.publish(task_to_publish);
    ros::spinOnce();
}



BT::NodeStatus SingleTaskDispatcher::tick()
{
  ROS_INFO_STREAM("SingleTaskDispatcher ticked");

  /* Input from behavior scheme */
  if(!SingleTaskDispatcher::getInput<std::string>("task_name",task_name_))
  {
    ROS_ERROR("Missing required input [task_name_]");
    throw BT::RuntimeError("Missing required input [task_name_]");
  }
  if(!SingleTaskDispatcher::getInput<std::string>("agent_name",agent_name_))
  {
    ROS_ERROR("Missing required input [task_name_]");
    throw BT::RuntimeError("Missing required input [task_name_]");
  }
  ROS_INFO_STREAM("Task Name: " << task_name_);
  ROS_INFO_STREAM("Agent Name: " << agent_name_);

  /* Nedded parameters */
  std::string task_feedback_topic_name;
  if (!pnh_.getParam("task_feedback_"+agent_name_,task_feedback_topic_name))
  {
    ROS_ERROR_STREAM("task_feedback_" << agent_name_ << " not defined");
    throw BT::RuntimeError("Missing required parameter task_feedback_");
  }
  std::string task_request_topic_name;
  if (!pnh_.getParam("task_request_"+agent_name_,task_request_topic_name))
  {
    ROS_ERROR_STREAM("task_request_" << agent_name_ <<" not defined");
    throw BT::RuntimeError("Missing required parameter task_request_");
  }

  /* Subscriber - Publisher */
  task_feedback_sub_.reset(new ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback> (nh_, task_feedback_topic_name,10));
  chatter_pub_ = nh_.advertise<task_planner_interface_msgs::MotionTaskExecutionRequestArray>(task_request_topic_name, 1000);

  /* Publish task request */
  publishTask(chatter_pub_,task_name_);

  /* Wait task response */
  while(!task_feedback_sub_->isANewDataAvailable() && ros::ok())
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  task_feedback_sub_->getData();

  ROS_INFO_STREAM("Task " << task_name_ << "executed.");
  return BT::NodeStatus::SUCCESS;
}

SingleTaskDispatcher::~SingleTaskDispatcher()
{
}
