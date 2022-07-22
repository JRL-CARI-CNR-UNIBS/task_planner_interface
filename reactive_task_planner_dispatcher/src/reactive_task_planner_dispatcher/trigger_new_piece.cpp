
#include "reactive_task_planner_dispatcher/trigger_new_piece.h"


TriggerNewPiece::TriggerNewPiece(const std::string &name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
{
  /* Nedded parameters */
  if (!nh_.getParam("trigger_topic_name",trigger_topic_name_))
  {
    ROS_ERROR_STREAM("trigger_topic_name not defined");
    throw BT::RuntimeError("Missing required parameter task_feedback_");
  }
  ROS_ERROR_STREAM(trigger_topic_name_);
  task_feedback_sub_.reset(new ros_helper::SubscriptionNotifier<mqtt_scene_integration::Fixture> (nh_, trigger_topic_name_,10));
}



BT::NodeStatus TriggerNewPiece::tick()
{
  ROS_INFO_STREAM("Waiting trigger of new piece...");
  while(!task_feedback_sub_->isANewDataAvailable() && ros::ok())
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  ROS_INFO_STREAM("Uscito");
  mqtt_scene_integration::Fixture fixture_msg = task_feedback_sub_->getData();
  TriggerNewPiece::setOutput("piece_output",fixture_msg.content);

  return BT::NodeStatus::SUCCESS;
}

TriggerNewPiece::~TriggerNewPiece()
{
}
