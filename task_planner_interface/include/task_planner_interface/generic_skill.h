#ifndef __generic_skill__
#define __generic_skill__

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <task_planner_interface_msgs/MotionTaskExecutionRequestArray.h>
#include <task_planner_interface_msgs/TaskResult.h>
#include <task_planner_interface_msgs/BasicSkill.h>

#include <task_planner_interface_msgs/TaskExecuteAction.h>  //For creation of TaskExecuteResult method


#include <task_planner_interface/agent_status.h>

///* for Mongo and BSON */
#include <cstdint>
#include <iostream>
#include <vector>
#include <bsoncxx/v_noabi/bsoncxx/json.hpp>
#include <mongocxx/v_noabi/mongocxx/client.hpp>
#include <mongocxx/v_noabi/mongocxx/stdx.hpp>
#include <mongocxx/v_noabi/mongocxx/uri.hpp>
#include <mongocxx/v_noabi/mongocxx/instance.hpp>

using bsoncxx::builder::stream::close_array;
using bsoncxx::builder::stream::close_document;
using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;
using bsoncxx::builder::stream::open_array;
//using bsoncxx::builder::stream::array;
using bsoncxx::builder::stream::open_document;

namespace taskPlannerInterface
{

namespace skills {

class GenericSkill
{
protected:
  std::string m_id;
  std::string m_type;
  std::string m_description;
  double m_duration_expected;
  double m_static_risk;
  std::vector<std::string> m_concurrent_tasks;

  std::string m_job_name;

  int m_outcome;
  double m_duration_real;  double m_duration_planned;
  double m_planning_time;
  double m_path_length;

  AgentStatusPtr m_agent_status;

  ros::NodeHandle m_nh;
  ros::ServiceClient m_skill_properties_client;

public:
  GenericSkill();
  GenericSkill(AgentStatusPtr agent_status_ptr);
  GenericSkill(const ros::NodeHandle& nh,AgentStatusPtr agent_status_ptr);

  virtual bool execute();
  virtual void init(const std::string& group_name);
  void setAgentStatus(AgentStatusPtr agent_status_ptr);
  virtual bool setPropertiesFromBSON(bsoncxx::stdx::optional<bsoncxx::document::value>& bson_doc);


  virtual bool setPropertiesFromService(task_planner_interface_msgs::BasicSkill task_properties);
  virtual bool setPropertiesFromService(std::string name, std::string type);

  bool setConcurrentTasksFromMsg(const task_planner_interface_msgs::MotionTaskExecutionRequestArray& msg);
  virtual bsoncxx::document::value getResultsAsBSON();

  task_planner_interface_msgs::TaskExecuteResult getResultsAsTaskExecuteResult(const std::string& group_name);

  void getResults(int& outcome, double& duration_real, double& duration_planned, double& planning_time, double& path_length) const;
  void print() const;  
  std::vector<std::string> getConcurrentTasks() const {return m_concurrent_tasks;};
  std::string getName() const {return m_id;};
  std::string getType() const {return m_type;};
  int getOutcome() const {return m_outcome;};


};
typedef std::shared_ptr<GenericSkill> GenericSkillPtr;

}


}

#endif
