#ifndef DOUBLE_TASK_DISPATCHER_H
#define DOUBLE_TASK_DISPATCHER_H

#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequestArray.h>
#include <task_planner_interface_msgs/MotionTaskExecutionFeedback.h>

class DoubleTaskDispatcher : public BT::SyncActionNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string task_name_first_agent_, task_name_second_agent_;
  std::string first_agent_name_, second_agent_name_;
  std::map<std::string,std::string> agents_requests_; //{first_agent_name_,second_agent_name_};
  std::map<std::string,bool> wait_agents_;


  std::map <std::string,std::shared_ptr<ros_helper::SubscriptionNotifier <task_planner_interface_msgs::MotionTaskExecutionFeedback>>> task_feedback_sub_;
  std::map <std::string, ros::Publisher> task_request_pub_;

  void publishTask(const ros::Publisher &pub,const std::string &task_name);
  template<typename T>
  inline void getInputWithCheck(const std::string &input_name, T &input_msg)
  {
    if(!DoubleTaskDispatcher::getInput(input_name,input_msg))
    {
      ROS_ERROR_STREAM("Missing required input [" << input_name << "]");
      throw BT::RuntimeError("Missing required input [" + input_name + "]");
    }
  }

public:
  DoubleTaskDispatcher(const std::string &name, const BT::NodeConfiguration& config);

  virtual ~DoubleTaskDispatcher() override;

  virtual BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return{BT::InputPort<std::string>("task_name_first_agent"),
          BT::InputPort<std::string>("task_name_second_agent"),
          BT::InputPort<std::string>("first_agent_name"),
          BT::InputPort<std::string>("second_agent_name"),
          BT::InputPort<bool>("wait_first_agent"),
          BT::InputPort<bool>("wait_second_agent"),
          BT::InputPort<std::string>("piece_first_agent_input"),
          BT::InputPort<std::string>("piece_second_agent_input"),
          BT::OutputPort<bool>("first_agent_state"),
          BT::OutputPort<bool>("second_agent_state"),
          BT::OutputPort<std::string>("piece_first_agent_output"),
          BT::OutputPort<std::string>("piece_second_agent_output"),
    };
  }

};


#endif //DOUBLE_TASK_DISPATCHER_H
