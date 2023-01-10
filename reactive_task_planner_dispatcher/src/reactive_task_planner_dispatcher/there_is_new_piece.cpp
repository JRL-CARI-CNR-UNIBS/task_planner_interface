
#include "reactive_task_planner_dispatcher/there_is_new_piece.h"


ThereIsNewPiece::ThereIsNewPiece(const std::string &name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
{
  /* Nedded parameters */
  if (!nh_.getParam("trigger_topic_name",trigger_topic_name_))
  {
    ROS_ERROR_STREAM("trigger_topic_name not defined");
    throw BT::RuntimeError("Missing required parameter task_feedback_");
  }
  std::string check_station_srv_name;
  if (!nh_.getParam("check_station_srv_name",check_station_srv_name))
  {
    ROS_ERROR_STREAM("check_station_srv_name not defined");
    throw BT::RuntimeError("Missing required parameter check_station_srv_name");
  }
  check_station_srv_client_ = nh_.serviceClient<std_srvs::Trigger>(check_station_srv_name);
}



BT::NodeStatus ThereIsNewPiece::tick()
{
  ROS_INFO_STREAM("---------------------------");
  ROS_INFO_STREAM("ThereIsNewPiece ticked");

  ROS_INFO_STREAM("Check trigger new piece...");

  std_srvs::Trigger check_trigger;
  if(check_station_srv_client_.call(check_trigger))
  {
    if(check_trigger.response.success)
    {
      ThereIsNewPiece::setOutput("piece_output",check_trigger.response.message);
      ROS_INFO_STREAM("There is new piece...");
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;

}

ThereIsNewPiece::~ThereIsNewPiece()
{
}
