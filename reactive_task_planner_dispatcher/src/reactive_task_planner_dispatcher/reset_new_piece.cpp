
#include "reactive_task_planner_dispatcher/reset_new_piece.h"


ResetNewPiece::ResetNewPiece(const std::string &name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
{

  std::string reset_piece_srv_name;
  if (!nh_.getParam("reset_piece_srv_name",reset_piece_srv_name))
  {
    ROS_ERROR_STREAM("reset_piece_srv_name not defined");
    throw BT::RuntimeError("Missing required parameter reset_piece_srv_name");
  }
  reset_new_piece_srv_client_ = nh_.serviceClient<std_srvs::Trigger>(reset_piece_srv_name);
  if(not reset_new_piece_srv_client_.waitForExistence(ros::Duration(1)))
    ROS_ERROR("Wait reset piece srv ");
}


BT::NodeStatus ResetNewPiece::tick()
{
  ROS_INFO_STREAM("---------------------------");
  ROS_INFO_STREAM("Reset Station 0");

  std_srvs::Trigger reset_state;
  if(reset_new_piece_srv_client_.call(reset_state))
  {
    ROS_INFO_STREAM("Reset Station 0 Done");
    return BT::NodeStatus::SUCCESS;   // Free
  }
  throw BT::RuntimeError("Service call failed. Not possible to go on");

}

ResetNewPiece::~ResetNewPiece()
{
}
