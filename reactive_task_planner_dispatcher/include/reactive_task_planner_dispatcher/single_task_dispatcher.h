#ifndef SINGLE_TASK_DISPATCHER_H
#define SINGLE_TASK_DISPATCHER_H

#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequestArray.h>
#include <task_planner_interface_msgs/MotionTaskExecutionFeedback.h>

class SingleTaskDispatcher : public BT::SyncActionNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string task_name_;
  std::string agent_name_;

  std::shared_ptr<ros_helper::SubscriptionNotifier <task_planner_interface_msgs::MotionTaskExecutionFeedback>> task_feedback_sub_;
  ros::Publisher chatter_pub_;

  void publishTask(const ros::Publisher &pub,
                                         const std::string &task_name);


public:
  SingleTaskDispatcher(const std::string &name, const BT::NodeConfiguration& config);
  virtual ~SingleTaskDispatcher() override;

  virtual BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
//    <std::string, PortInfo>
    return{BT::InputPort<std::string>("task_name"),BT::InputPort<std::string>("agent_name")};
  }


};
//class FixtureCheck : public BT::SyncActionNode
//{
//public:
//    FixtureCheck(const std::string& name);

//    BT::NodeStatus tick() override;

//private:
//    ros::NodeHandle n_;
//    ros::ServiceClient skill_exec_clnt_;
//};

#endif // FIXTURE_CHECK_H