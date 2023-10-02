#ifndef __TASK_PLANNER_PICKPLACE_54223__
#define __TASK_PLANNER_PICKPLACE_54223__

//#include "Eigen/Dense"
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/PickObjectsAction.h>
#include <manipulation_msgs/PlaceObjectsAction.h>
#include <manipulation_msgs/GoToAction.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequestArray.h>
#include <task_planner_interface/agent_status.h>
#include <task_planner_interface/generic_skill.h>
#include <task_planner_interface_msgs/PickPlaceSkill.h>

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

class PickPlaceSkill : public GenericSkill
{
protected:
  std::vector<std::string> m_pick_goal;
  std::vector<std::string> m_place_goal;
  boost::shared_ptr<actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction> > m_pick_ac;
  boost::shared_ptr<actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction> > m_place_ac;
  bool m_have_additional_properties = false;
  std::map<std::string,std::string> m_pick_additional_properties;
  std::map<std::string,std::string> m_place_additional_properties;

public:
  PickPlaceSkill();

  PickPlaceSkill(const AgentStatusPtr& agent_status_ptr);
  PickPlaceSkill(const AgentStatusPtr& agent_status_ptr,
                 const std::map<std::string, std::string>& pick_additional_properties,
                 const std::map<std::string, std::string>& place_additional_properties);

  bool execute();
  void init(const std::string& arg);
  bool setPropertiesFromBSON(bsoncxx::stdx::optional<bsoncxx::document::value>& bson_doc);
  bool setPropertiesFromService(task_planner_interface_msgs::PickPlaceSkill task_properties);
  bool setPropertiesFromService(std::string name,std::string type);

  void setPickGoal(std::vector<std::string>& arg);
  void setPlaceGoal(std::vector<std::string>& arg);

};
typedef std::shared_ptr<PickPlaceSkill> PickPlaceSkillPtr;

}


}

#endif
