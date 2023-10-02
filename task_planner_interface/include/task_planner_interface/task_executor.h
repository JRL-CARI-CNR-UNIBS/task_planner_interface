#ifndef __TASK_EXECUTOR_H__
#define __TASK_EXECUTOR_H__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <boost/shared_ptr.hpp>
#include <actionlib/client/simple_action_client.h>

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
#include <task_planner_interface_msgs/MotionTaskExecutionRequest.h>

#include <task_planner_interface_msgs/TaskExecuteAction.h>

#include <task_planner_interface_msgs/TaskResult.h>
#include <task_planner_interface_msgs/TaskType.h>
#include <task_planner_interface_msgs/BasicSkill.h>
#include <task_planner_interface_msgs/PickPlaceSkill.h>

#include<std_srvs/Trigger.h>

namespace taskPlannerInterface
{


class TaskExecutor
{
protected:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_pnh;

  std::string m_action_name;
  std::string m_group_name;

  actionlib::SimpleActionServer<task_planner_interface_msgs::TaskExecuteAction> m_as;
  task_planner_interface_msgs::TaskExecuteFeedback m_feedback;
  task_planner_interface_msgs::TaskExecuteResult m_result;

  ros::ServiceClient m_skill_type_client;

  taskPlannerInterface::AgentStatusPtr m_agent_status;
  std::map<std::string, taskPlannerInterface::skills::GenericSkillPtr> m_skills;        //Non sono sicuro sia corretto sono già dentro il namespace

  ros::Duration m_t_total;


  std::string m_home_position;
  std::string m_retry_position;
  bool m_go_home_after_execution;

  taskPlannerInterface::skills::GoToSkillPtr m_basic_goto_skill;

  ros::ServiceServer m_reset_agent_state_srv;


  void executeTask(const task_planner_interface_msgs::TaskExecuteGoalConstPtr& goal);
  bool checkSkillType(const std::string name, std::string& skill_type);
  bool resetAgentState(std_srvs::Trigger::Request&  req,
                       std_srvs::Trigger::Response& res);
  bool getAdditionalProperties(const std::string& skill_name,
                               std::map<std::string, std::string>& additional_properties);
public:
  TaskExecutor(const ros::NodeHandle& nh,
               const ros::NodeHandle& pnh,
               const std::string action_name,
               const std::string group_name,
               const std::string home_position,
               const std::string retry_position,
               const bool go_home_after_execution,
               const std::string reset_agent_srv_name);


};




}

#endif
