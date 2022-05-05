#include <task_planner_interface/place_skill.h>

namespace taskPlannerInterface
{

namespace skills {

#define SUCCESS 1
#define FAILURE 0

PlaceSkill::PlaceSkill(){}

bool PlaceSkill::execute()
{
  if (m_agent_status->getObjectInHandType().empty())
  {
    ROS_INFO_STREAM("Command " << m_id << ": no object in hand to place");
    m_outcome = FAILURE;
    m_duration_real=0.0;
    m_duration_planned=0.0;
    m_planning_time=0.0;
    m_path_length=0.0;
    return false;
  }
  else
  {
    manipulation_msgs::PlaceObjectsGoal place_goal;
    place_goal.object_name=m_agent_status->getObjectInHandId();
    //place_goal.tool_id="gripper_fake";
    //place_goal.property_exec_id="open_100";
    //place_goal.property_post_exec_id="open";
    //place_goal.job_exec_name=m_job_name;
    //place_goal.object_name=m_agent_status->getObjectInHandType();
    place_goal.slots_group_names=m_place_goal;

    ROS_ERROR_STREAM("goal place:");
    for (auto it=m_place_goal.begin(); it!=m_place_goal.end(); ++it)
    {
      std::cout << "- " << it->data() << std::endl;
    }

    m_place_ac->sendGoalAndWait(place_goal);
//    manipulation_msgs::PlaceObjectsResultConstPtr action_res= m_place_ac->getResult();
//    action_res->estimated_hrc_execution_duration.toSec();

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
      m_duration_real=m_place_ac->getResult()->actual_duration.toSec();
      m_duration_planned=m_place_ac->getResult()->expected_execution_duration.toSec();
      m_planning_time=m_place_ac->getResult()->planning_duration.toSec();
      m_path_length=m_place_ac->getResult()->path_length;
    }
  }
  return (bool)m_outcome;
}

void PlaceSkill::init(const std::string& group_name)
{
  if (m_job_name.empty())
  {
    m_job_name="place";
  }

  m_outcome=FAILURE;
  m_place_ac.reset(new actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction>("/outbound_place_server/"+group_name+"/"+m_job_name));
  m_place_ac->waitForServer();
  ROS_INFO("Initilized place skill for movegroup %s", group_name.c_str());
}

bool PlaceSkill::setPropertiesFromBSON(bsoncxx::stdx::optional<bsoncxx::document::value>& bson_doc){
  if (!GenericSkill::setPropertiesFromBSON(bson_doc))
    return false;

  m_place_goal.clear();
  bsoncxx::document::element ele_goal{bson_doc->view()["goal"]};
  bsoncxx::array::view ele_goal_view{ele_goal.get_array().value};
  for (bsoncxx::array::element subdocument : ele_goal_view){
      //std::cout<< subdocument["FunctionalArea"].get_utf8().value << std::endl;
      m_place_goal.push_back(subdocument.get_utf8().value.to_string());
  }

  //m_place_goal=ele_goal.get_utf8().value.to_string();

  try
  {
    bsoncxx::document::element ele_job{bson_doc->view()["job_name"]};
    m_job_name=ele_job.get_utf8().value.to_string();
  }
  catch (...)
  {
    m_job_name="place";
  }

  return true;  }

bool PlaceSkill::setPropertiesFromService(task_planner_interface_msgs::BasicSkill task_properties)
{
  m_place_goal.clear();
  m_place_goal = task_properties.response.goal;
//  goal_array = task_properties.response.goal;

//  for(std::string single_goal : goal_array)
//  {
//      m_place_goal.push_back(single_goal);
//  }

  if(task_properties.response.job_name.empty())
  {
      m_job_name = "place";
  }
  else
  {
      m_job_name = task_properties.response.job_name;
  }

  ROS_INFO("ID: %s", m_id.c_str());
  ROS_INFO("TYPE: %s", m_type.c_str());
  ROS_INFO("JOB_NAME: %s", m_job_name.c_str());

  return true;
}

bool PlaceSkill::setPropertiesFromService(std::string name, std::string type)
{
  GenericSkill::setPropertiesFromService(name,type);
  m_place_goal.clear();

  task_planner_interface_msgs::BasicSkill srv;
  srv.request.name = name;

  if(m_skill_properties_client.call(srv))
  {
      if(!srv.response.error)   // If no error
      {
          ROS_INFO("Task properties retrieved correctly");
          m_place_goal = srv.response.goal;

          if(srv.response.job_name.empty())
          {
              m_job_name = "place";
          }
          else
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


void PlaceSkill::setGoal(std::vector<std::string>& arg){ m_place_goal=arg; }

} // end namespace

} //end namespace
