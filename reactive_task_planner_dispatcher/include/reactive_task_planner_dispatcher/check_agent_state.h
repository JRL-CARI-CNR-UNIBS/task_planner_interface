#ifndef CHECK_AGENT_STATE_H
#define CHECK_AGENT_STATE_H

#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <std_srvs/Trigger.h>

class CheckAgentState : public BT::ConditionNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string agent_name_;
	

  ros::ServiceClient check_agent_state_srv_client_;

public:
  CheckAgentState(const std::string &name, const BT::NodeConfiguration& config);
  virtual ~CheckAgentState() override;

  virtual BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return{BT::InputPort<std::string>("agent_name") };
  }


};

#endif // WAIT_AGENT_FEEDBACK_H
