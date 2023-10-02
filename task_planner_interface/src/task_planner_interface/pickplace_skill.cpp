#include <task_planner_interface/pickplace_skill.h>

namespace taskPlannerInterface
{

namespace skills {

#define SUCCESS 1
#define FAILURE 0

PickPlaceSkill::PickPlaceSkill(){
    m_skill_properties_client =  m_nh.serviceClient<task_planner_interface_msgs::PickPlaceSkill>("mongo_handler/get_task_properties_pickplace");
    m_have_additional_properties = false;
}

PickPlaceSkill::PickPlaceSkill(const AgentStatusPtr& agent_status_ptr):GenericSkill(agent_status_ptr)
{
  m_skill_properties_client = m_nh.serviceClient <task_planner_interface_msgs::PickPlaceSkill>("mongo_handler/get_task_properties_pickplace");
  m_have_additional_properties = false;
}

PickPlaceSkill::PickPlaceSkill(const AgentStatusPtr& agent_status_ptr,
                               const std::map<std::string, std::string>& pick_additional_properties,
                               const std::map<std::string, std::string>& place_additional_properties):GenericSkill(agent_status_ptr)
{
  m_skill_properties_client = m_nh.serviceClient <task_planner_interface_msgs::PickPlaceSkill>("mongo_handler/get_task_properties_pickplace");
  m_have_additional_properties = true;
  m_pick_additional_properties = pick_additional_properties;
  m_place_additional_properties = place_additional_properties;
}

bool PickPlaceSkill::execute()
{
  m_outcome = FAILURE;
  m_duration_real=0.0;
  m_duration_planned=0.0;
  m_planning_time=0.0;
  m_path_length=0.0;
  ROS_INFO_STREAM("EXECUTING PICKPLACE: "<< m_pick_goal.size());
  for(std::string &goal: m_pick_goal)
  {
    ROS_INFO_STREAM("Pick goal" << goal);
  }

  for (auto it=m_pick_goal.begin(); it!=m_pick_goal.end(); ++it)
  {
    if (!m_agent_status->getObjectInHandType().compare(it->data()))
    {
      ROS_INFO_STREAM("Command " << m_id << ": requested object already in hand");
      m_outcome = SUCCESS;
    }
  }

  if ( m_outcome = SUCCESS )
  {
    if (m_agent_status->getObjectInHandType().empty())
    {
      manipulation_msgs::PickObjectsGoal pick_goal;
      pick_goal.object_types = m_pick_goal;
      pick_goal.tool_id="robotiq_gripper";
      pick_goal.property_pre_exec_id="open_60";
      pick_goal.property_exec_id="close";
      pick_goal.job_exec_name="pick";
      for(std::string &pick_goal: pick_goal.object_types)
      {
        ROS_INFO_STREAM("--- PICK GOAL "<< pick_goal);
      }
      for(std::string &pick_goal: m_pick_goal)
      {
        ROS_INFO_STREAM("--- PICK GOAL "<< pick_goal);
      }
      m_pick_ac->sendGoalAndWait(pick_goal);

      if (m_pick_ac->getResult()->result<0)
      {
        ROS_ERROR_STREAM("Command " << m_id << ": unable to pick any of the object types:");
        for (auto it=m_pick_goal.begin(); it!=m_pick_goal.end(); ++it)
        {
          std::cout << "- " << it->data() << std::endl;
        }

        m_outcome = FAILURE;
        return false;
      }
      else
      {
        m_agent_status->setObjectInHand(m_pick_ac->getResult()->object_name,m_pick_ac->getResult()->object_type);
        ROS_INFO_STREAM("Command " << m_id << ": well done! I picked it, id=" << m_pick_ac->getResult()->object_name);
        m_outcome = SUCCESS;
        m_duration_real=m_pick_ac->getResult()->actual_duration.toSec();
        m_duration_planned=m_pick_ac->getResult()->expected_execution_duration.toSec();
        m_planning_time=m_pick_ac->getResult()->planning_duration.toSec();
        m_path_length=m_pick_ac->getResult()->path_length;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Command " << m_id << ": unable to pick because another object has already been picked");
      m_outcome = FAILURE;
      return false;
    }
  }

  manipulation_msgs::PlaceObjectsGoal place_goal;
  place_goal.object_name=m_agent_status->getObjectInHandId();
  place_goal.tool_id="robotiq_gripper";
  place_goal.property_exec_id="open_100";
  place_goal.property_post_exec_id="open";
  place_goal.job_exec_name="place";
  //place_goal.object_type=m_agent_status->getObjectInHandType();
  place_goal.slots_group_names=m_place_goal;
  m_place_ac->sendGoalAndWait(place_goal);
  if (m_place_ac->getResult()->result<0)
  {
    ROS_ERROR_STREAM("Command " << m_id << ": unable to place");
    m_outcome = FAILURE;
    m_duration_real=0.0;
    m_duration_planned=0.0;
    m_planning_time=0.0;
    m_path_length=0.0;
    return false;
  }
  else
  {
    ROS_INFO_STREAM("Command " << m_id << ": well done! I placed it, id=" << m_agent_status->getObjectInHandId());
    m_agent_status->setObjectInHand("","");
    m_outcome = SUCCESS;
    m_duration_real+=m_place_ac->getResult()->actual_duration.toSec();
    m_duration_planned+=m_place_ac->getResult()->expected_execution_duration.toSec();
    m_planning_time+=m_place_ac->getResult()->planning_duration.toSec();
    m_path_length+=m_place_ac->getResult()->path_length;
  }
  return (bool)m_outcome;
}

void PickPlaceSkill::init(const std::string& arg)
{
  std::string group_name=arg;
  m_outcome=FAILURE;
  m_pick_ac.reset(new actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction>("/inbound_pick_server/"+group_name+"/pick"));
  m_place_ac.reset(new actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction>("/outbound_place_server/"+group_name+"/place"));
  m_pick_ac->waitForServer();
  m_place_ac->waitForServer();
//  m_pick_goal.clear();
//  m_place_goal.clear();
  ROS_INFO("Initilized pickplace skill for movegroup %s", group_name.c_str());
}

bool PickPlaceSkill::setPropertiesFromBSON(bsoncxx::stdx::optional<bsoncxx::document::value>& bson_doc){
  if (!GenericSkill::setPropertiesFromBSON(bson_doc))
    return false;

  m_pick_goal.clear();
  bsoncxx::document::element ele_pick_goal{bson_doc->view()["pick_goal"]};
  bsoncxx::array::view ele_pick_goal_view{ele_pick_goal.get_array().value};
  for (bsoncxx::array::element subdocument : ele_pick_goal_view){
      //std::cout<< subdocument["FunctionalArea"].get_utf8().value << std::endl;
      m_pick_goal.push_back(subdocument.get_utf8().value.to_string());
  }

  m_place_goal.clear();
  bsoncxx::document::element ele_place_goal{bson_doc->view()["place_goal"]};
  bsoncxx::array::view ele_place_goal_view{ele_place_goal.get_array().value};
  for (bsoncxx::array::element subdocument : ele_place_goal_view){
      //std::cout<< subdocument["FunctionalArea"].get_utf8().value << std::endl;
      m_place_goal.push_back(subdocument.get_utf8().value.to_string());
  }

  //m_pick_goal=ele_pick_goal.get_utf8().value.to_string();
  //m_place_goal=ele_place_goal.get_utf8().value.to_string();
  return true;
}

bool PickPlaceSkill::setPropertiesFromService(task_planner_interface_msgs::PickPlaceSkill task_properties)
{
  m_pick_goal.clear();
  m_place_goal.clear();

  m_pick_goal = task_properties.response.pick_goal;
  m_place_goal = task_properties.response.place_goal;

  return true;
}

bool PickPlaceSkill::setPropertiesFromService(std::string name, std::string type)
{
  GenericSkill::setPropertiesFromService(name,type);
  m_pick_goal.clear();
  m_place_goal.clear();

  task_planner_interface_msgs::PickPlaceSkill srv;
  srv.request.name = name;

  if(m_skill_properties_client.call(srv))
  {
      if(!srv.response.error)   // If no error
      {
          ROS_INFO("Task properties retrieved correctly");
          m_pick_goal = srv.response.pick_goal;
          m_place_goal = srv.response.place_goal;
          for(std::string &goal: m_pick_goal)
          {
            ROS_INFO_STREAM("Pick goal" << goal);
          }
          for(std::string &goal: m_place_goal)
          {
            ROS_INFO_STREAM("Pick goal" << goal);
          }
          if(!srv.response.job_name.empty())
          {
              m_job_name = srv.response.job_name;
          }
          return true;
      }
      else
      {
          ROS_ERROR("Task type does not exist");
      }
  }
  else
  {
      ROS_ERROR("Rosservice call for retrive properties failed");
  }
  return false;
}


void PickPlaceSkill::setPickGoal(std::vector<std::string>& arg)
{
  m_pick_goal=arg;
}

void PickPlaceSkill::setPlaceGoal(std::vector<std::string>& arg)
{
  m_place_goal=arg;
}

} // end namespace

} //end namespace
