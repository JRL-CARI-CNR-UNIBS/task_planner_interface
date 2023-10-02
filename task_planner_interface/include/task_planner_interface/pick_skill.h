#ifndef __TASK_PLANNER_PICK_4739__
#define __TASK_PLANNER_PICK_4739__

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/PickObjectsAction.h>
#include <manipulation_msgs/GoToAction.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequestArray.h>
#include <task_planner_interface/agent_status.h>
#include <task_planner_interface/generic_skill.h>

namespace taskPlannerInterface
{

namespace skills {

class PickSkill : public GenericSkill
{
protected:
  std::vector<std::string> m_pick_goal;
  boost::shared_ptr<actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction> > m_pick_ac;
  bool m_have_additional_properties = false;
  std::map<std::string,std::string> m_additional_properties;

public:
  PickSkill();
  PickSkill(const AgentStatusPtr& agent_status_ptr):GenericSkill(agent_status_ptr){
    m_have_additional_properties = false;
  }
  PickSkill(const AgentStatusPtr& agent_status_ptr,
            const std::map<std::string, std::string>& additional_properties);

  bool execute();
  void init(const std::string& group_name);
  bool setPropertiesFromBSON(bsoncxx::stdx::optional<bsoncxx::document::value>& bson_doc);
  bool setPropertiesFromService(task_planner_interface_msgs::BasicSkill task_properties);
  bool setPropertiesFromService(std::string name,std::string type);

  void setGoal(std::vector<std::string>& arg);

};
typedef std::shared_ptr<PickSkill> PickSkillPtr;


}

}

#endif
