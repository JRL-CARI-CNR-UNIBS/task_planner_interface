
#include "reactive_task_planner_dispatcher/check_agent_state.h"


CheckAgentState::CheckAgentState(const std::string &name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
{
  if(!CheckAgentState::getInput("agent_name",agent_name_))
  {
    ROS_ERROR_STREAM("Missing required input [agent_name]");
    throw BT::RuntimeError("Missing required input [agent_name]");
  }
  std::string check_agent_state_srv_name;
  if (!nh_.getParam("check_agent_state_prefix_srv_name",check_agent_state_srv_name))
  {
    ROS_ERROR_STREAM("check_agent_state_prefix_srv_name not defined");
    throw BT::RuntimeError("Missing required parameter check_agent_state_prefix_srv_name");
  }
  check_agent_state_srv_client_ = nh_.serviceClient<std_srvs::Trigger>(check_agent_state_srv_name+"_"+agent_name_);
  if(not check_agent_state_srv_client_.waitForExistence(ros::Duration(1)))
    ROS_ERROR("Wait check agent state");
}


BT::NodeStatus CheckAgentState::tick()
{
  ROS_INFO_STREAM("---------------------------");
  ROS_INFO_STREAM("Check " << agent_name_<<" state");

  std_srvs::Trigger check_state;
  if(check_agent_state_srv_client_.call(check_state))
  {
    if(check_state.response.success)
    {
      ROS_INFO_STREAM("Agent " << agent_name_<<" busy (failure)");
      return BT::NodeStatus::FAILURE; // Busy
    }
    ROS_INFO_STREAM("Agent "<< agent_name_ <<" free (success)");
    return BT::NodeStatus::SUCCESS;   // Free
  }
  throw BT::RuntimeError("Service call failed. Not possible to go on");

}

CheckAgentState::~CheckAgentState()
{
}
