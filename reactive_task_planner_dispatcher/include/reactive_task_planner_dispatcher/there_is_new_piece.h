#ifndef THERE_IS_NEW_PIECE_H
#define THERE_IS_NEW_PIECE_H

#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <task_planner_interface_msgs/MotionTaskExecutionFeedback.h>
#include <std_srvs/Trigger.h>

class ThereIsNewPiece : public BT::ConditionNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string trigger_topic_name_;

  ros::ServiceClient check_station_srv_client_;

public:
  ThereIsNewPiece(const std::string &name, const BT::NodeConfiguration& config);
  virtual ~ThereIsNewPiece() override;

  virtual BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return{BT::OutputPort<std::string>("piece_output") };
  }


};

#endif // WAIT_AGENT_FEEDBACK_H
