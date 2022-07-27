#ifndef RESET_NEW_PIECE_H
#define RESET_NEW_PIECE_H

#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <std_srvs/Trigger.h>

class ResetNewPiece : public BT::ConditionNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

	
  ros::ServiceClient reset_new_piece_srv_client_;

public:
  ResetNewPiece(const std::string &name, const BT::NodeConfiguration& config);
  virtual ~ResetNewPiece() override;

  virtual BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return{};
  }


};

#endif // RESET_NEW_PIECE_H
