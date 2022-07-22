
#include "reactive_task_planner_dispatcher/wait_agent_feedback.h"


WaitAgentFeedback::WaitAgentFeedback(const std::string &name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
{
  /* Input from behavior scheme */

  if(!WaitAgentFeedback::getInput<std::string>("agent_name",agent_name_))
  {
    ROS_ERROR("Missing required input [task_name_]");
    throw BT::RuntimeError("Missing required input [task_name_]");
  }
  std::string piece_input;
  if(WaitAgentFeedback::getInput("piece_input",piece_input))
  {
    WaitAgentFeedback::setOutput("piece_output",piece_input);
  }
  ROS_INFO_STREAM("Agent Name: " << agent_name_);

  /* Nedded parameters */
  if (!nh_.getParam("task_feedback_"+agent_name_,task_feedback_topic_name_))
  {
    ROS_ERROR_STREAM("task_feedback_" << agent_name_ << " not defined");
    throw BT::RuntimeError("Missing required parameter task_feedback_");
  }

}



BT::NodeStatus WaitAgentFeedback::tick()
{
  ROS_INFO_STREAM("---------------------------");
  ROS_INFO_STREAM("Wait Agent Feedback ticked");

  /* Subscriber - Publisher */
  task_feedback_sub_.reset(new ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback> (nh_, task_feedback_topic_name_,10));

  /* Wait task response */
  ROS_INFO_STREAM("Waiting task feedback...");
  while(!task_feedback_sub_->isANewDataAvailable() && ros::ok())
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  task_feedback_sub_->getData();

  ROS_INFO_STREAM("Feedback arrived.");
  return BT::NodeStatus::SUCCESS;
}

WaitAgentFeedback::~WaitAgentFeedback()
{
}
