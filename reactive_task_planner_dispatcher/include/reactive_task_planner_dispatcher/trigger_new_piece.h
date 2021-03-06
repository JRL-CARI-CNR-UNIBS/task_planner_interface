#ifndef TRIGGER_NEW_PIECE_H
#define TRIGGER_NEW_PIECE_H

#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <task_planner_interface_msgs/MotionTaskExecutionFeedback.h>
#include <mqtt_scene_integration/Fixture.h>

class TriggerNewPiece : public BT::ConditionNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string trigger_topic_name_;

  std::shared_ptr<ros_helper::SubscriptionNotifier <mqtt_scene_integration::Fixture>> task_feedback_sub_;


public:
  TriggerNewPiece(const std::string &name, const BT::NodeConfiguration& config);
  virtual ~TriggerNewPiece() override;

  virtual BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return{BT::OutputPort<std::string>("piece_output") };
  }


};

#endif // WAIT_AGENT_FEEDBACK_H
