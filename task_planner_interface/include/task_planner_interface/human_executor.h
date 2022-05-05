#ifndef HUMAN_EXECUTOR_H
#define HUMAN_EXECUTOR_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <subscription_notifier/subscription_notifier.h>


#include <boost/shared_ptr.hpp>

#include <task_planner_interface/agent_status.h>

#include <task_planner_interface/generic_skill.h>
#include <task_planner_interface/pick_skill.h>
#include <task_planner_interface/place_skill.h>
#include <task_planner_interface/go_to_skill.h>
#include <task_planner_interface/pickplace_skill.h>

#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <task_planner_interface_msgs/MotionTaskExecutionRequestArray.h>
#include <task_planner_interface_msgs/MotionTaskExecutionFeedback.h>


#include <task_planner_interface_msgs/TaskExecuteAction.h>

#include <task_planner_interface_msgs/TaskResult.h>
#include <task_planner_interface_msgs/TaskType.h>
#include <task_planner_interface_msgs/BasicSkill.h>
#include <task_planner_interface_msgs/PickPlaceSkill.h>

namespace taskPlannerInterface
{


class HumanExecutor
{
protected:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_pnh;

  std::string m_action_name;
  std::string m_group_name;

  actionlib::SimpleActionServer<task_planner_interface_msgs::TaskExecuteAction> m_as;
  task_planner_interface_msgs::TaskExecuteFeedback m_feedback;
  task_planner_interface_msgs::TaskExecuteResult m_result;

  /*Services*/
  ros::ServiceClient m_skill_type_client;
  ros::ServiceClient m_skill_properties_client;


  ros::Publisher m_pub_task_request;
  std::shared_ptr<ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback>> m_sub_task_result;

  //ros::Subscriber m_sub_task_result;


  void executeTask(const task_planner_interface_msgs::TaskExecuteGoalConstPtr& goal);
  bool checkSkillType(const std::string name, std::string& skill_type);
  bool setPropertiesFromService(std::string name,std::string type,std::string& description);

public:
  HumanExecutor(const ros::NodeHandle& nh,
                const ros::NodeHandle& pnh,
                const std::string action_name,
                const std::string group_name);


};




}

#endif // HUMAN_EXECUTOR_H
