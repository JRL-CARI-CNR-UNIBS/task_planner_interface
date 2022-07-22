
#include "reactive_task_planner_dispatcher/there_is_new_piece.h"


ThereIsNewPiece::ThereIsNewPiece(const std::string &name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
{
  /* Nedded parameters */
  if (!nh_.getParam("trigger_topic_name",trigger_topic_name_))
  {
    ROS_ERROR_STREAM("trigger_topic_name not defined");
    throw BT::RuntimeError("Missing required parameter task_feedback_");
  }
  task_feedback_sub_.reset(new ros_helper::SubscriptionNotifier<mqtt_scene_integration::Fixture> (nh_, trigger_topic_name_,10));

}



BT::NodeStatus ThereIsNewPiece::tick()
{
  ROS_INFO_STREAM("Check trigger new piece...");
  if(task_feedback_sub_->isANewDataAvailable())
  {
    mqtt_scene_integration::Fixture fixture_msg = task_feedback_sub_->getData();
    ThereIsNewPiece::setOutput("piece_output",fixture_msg.content);
    ROS_INFO_STREAM("There is new piece...");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    ROS_INFO_STREAM("No new piece...");
    return BT::NodeStatus::FAILURE;
  }

}

ThereIsNewPiece::~ThereIsNewPiece()
{
}
