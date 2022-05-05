#ifndef __TASK_PLANNER_GO_TO_54123__
#define __TASK_PLANNER_GO_TO_54123__

//#include "Eigen/Dense"
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/GoToAction.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequestArray.h>
#include <task_planner_interface/agent_status.h>
#include <task_planner_interface/generic_skill.h>

///* for Mongo */
//#include <cstdint>
//#include <iostream>
//#include <vector>
//#include <bsoncxx/v_noabi/bsoncxx/json.hpp>
//#include <mongocxx/v_noabi/mongocxx/client.hpp>
//#include <mongocxx/v_noabi/mongocxx/stdx.hpp>
//#include <mongocxx/v_noabi/mongocxx/uri.hpp>
//#include <mongocxx/v_noabi/mongocxx/instance.hpp>

//using bsoncxx::builder::stream::close_array;
//using bsoncxx::builder::stream::close_document;
//using bsoncxx::builder::stream::document;
//using bsoncxx::builder::stream::finalize;
//using bsoncxx::builder::stream::open_array;
////using bsoncxx::builder::stream::array;
//using bsoncxx::builder::stream::open_document;

namespace taskPlannerInterface
{

namespace skills {

class GoToSkill : public GenericSkill
{
protected:
  std::vector<std::string> m_goal_location;
  boost::shared_ptr< actionlib::SimpleActionClient<manipulation_msgs::GoToAction> > m_goto_ac;
public:
  GoToSkill();
  GoToSkill(const AgentStatusPtr& agent_status_ptr):GenericSkill(agent_status_ptr){};
  bool execute();
  void init(const std::string& group_name);
  bool setPropertiesFromBSON(bsoncxx::stdx::optional<bsoncxx::document::value>& bson_doc);
  bool setPropertiesFromService(task_planner_interface_msgs::BasicSkill task_properties);
  bool setPropertiesFromService(std::string name,std::string type);

  void setGoal(std::vector<std::string>& arg);
  bool sendDirectLocation(const std::string& goal_location);

};
typedef std::shared_ptr<GoToSkill> GoToSkillPtr;


}


}

#endif
