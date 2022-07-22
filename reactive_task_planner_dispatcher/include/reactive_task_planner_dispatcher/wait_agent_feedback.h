#ifndef WAIT_AGENT_FEEDBACK_H
#define WAIT_AGENT_FEEDBACK_H

#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <task_planner_interface_msgs/MotionTaskExecutionFeedback.h>

class WaitAgentFeedback : public BT::SyncActionNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string agent_name_;
  std::string task_feedback_topic_name_;

  std::shared_ptr<ros_helper::SubscriptionNotifier <task_planner_interface_msgs::MotionTaskExecutionFeedback>> task_feedback_sub_;



public:
  WaitAgentFeedback(const std::string &name, const BT::NodeConfiguration& config);
  virtual ~WaitAgentFeedback() override;

  virtual BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return{BT::InputPort<std::string>("agent_name"),
          BT::InputPort<std::string>("piece_input"),
          BT::OutputPort<std::string>("piece_output") };
  }


};

#endif // WAIT_AGENT_FEEDBACK_H
