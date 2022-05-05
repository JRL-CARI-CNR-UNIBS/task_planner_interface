#include <task_planner_interface/pick_skill.h>

namespace taskPlannerInterface
{

namespace skills {

#define SUCCESS 1
#define FAILURE 0

PickSkill::PickSkill(){}

bool PickSkill::execute()
{
  m_outcome = FAILURE;
  m_duration_real=0.0;
  m_duration_planned=0.0;
  m_planning_time=0.0;
  m_path_length=0.0;


  for (auto it=m_pick_goal.begin(); it!=m_pick_goal.end(); ++it)
  {
    if (!m_agent_status->getObjectInHandType().compare(it->data()))
    {
      ROS_INFO_STREAM("Command " << m_id << ": requested object already in hand");
      m_outcome = SUCCESS;
      return (bool)m_outcome;
    }
  }

  if (m_agent_status->getObjectInHandType().empty())
  {
    manipulation_msgs::PickObjectsGoal pick_goal;
    pick_goal.object_types=m_pick_goal;
    //pick_goal.tool_id="robotiq_gripper";
    //pick_goal.property_pre_exec_id="open_100";
    //pick_goal.property_exec_id="close";
    //pick_goal.job_exec_name=m_job_name;
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
      ROS_INFO_STREAM("Command " << m_id << ": well done! I picked it, id=" << m_pick_ac->getResult()->object_name << "; type= " << m_pick_ac->getResult()->object_type << "; inbound_box= " << m_pick_ac->getResult()->inbound_box);
      m_duration_real=m_pick_ac->getResult()->actual_duration.toSec();
      m_duration_planned=m_pick_ac->getResult()->expected_execution_duration.toSec();
      m_planning_time=m_pick_ac->getResult()->planning_duration.toSec();
      m_path_length=m_pick_ac->getResult()->path_length;
      m_outcome = SUCCESS;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Command " << m_id << ": unable to pick because another object has already been picked");
    m_outcome = FAILURE;
    return false;
  }
  return (bool)m_outcome;
}

void PickSkill::init(const std::string& group_name)
{
  if (m_job_name.empty())
  {
    m_job_name="pick";
  }

  m_outcome=FAILURE;
  m_pick_ac.reset(new actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction>("/inbound_pick_server/"+group_name+"/"+m_job_name));
  m_pick_ac->waitForServer();
  ROS_INFO("Initilized pick skill for movegroup %s", group_name.c_str());
}

bool PickSkill::setPropertiesFromBSON(bsoncxx::stdx::optional<bsoncxx::document::value>& bson_doc){
  GenericSkill::setPropertiesFromBSON(bson_doc);

  bsoncxx::document::element ele_goal{bson_doc->view()["goal"]};

  //m_pick_goal=ele_goal.get_utf8().value.to_string();

  m_pick_goal.clear();
  bsoncxx::array::view ele_goal_view{ele_goal.get_array().value};
  for (bsoncxx::array::element subdocument : ele_goal_view){
      //std::cout<< subdocument["FunctionalArea"].get_utf8().value << std::endl;
      m_pick_goal.push_back(subdocument.get_utf8().value.to_string());
  }

  try
  {
    bsoncxx::document::element ele_job{bson_doc->view()["job_name"]};
    m_job_name=ele_job.get_utf8().value.to_string();
  }
  catch (...)
  {
    m_job_name="pick";
  }

  return true;
}

bool PickSkill::setPropertiesFromService(task_planner_interface_msgs::BasicSkill task_properties)
{
  m_pick_goal.clear();
  m_pick_goal = task_properties.response.goal;

//  goal_array = task_properties.response.goal;

//  for(std::string single_goal : goal_array)
//  {
//      m_pick_goal.push_back(single_goal);
//  }

  if(task_properties.response.job_name.empty())
  {
      m_job_name = "pick";
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
bool PickSkill::setPropertiesFromService(std::string name,std::string type)
{
  GenericSkill::setPropertiesFromService(name,type);
  m_pick_goal.clear();

  task_planner_interface_msgs::BasicSkill srv;
  srv.request.name = name;

  if(m_skill_properties_client.call(srv))
  {
      if(!srv.response.error)   // If no error
      {
          ROS_INFO("Task properties retrieved correctly");
          m_pick_goal = srv.response.goal;
          if(srv.response.job_name.empty())
          {
              m_job_name = "pick";
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

void PickSkill::setGoal(std::vector<std::string>& arg){ m_pick_goal=arg; }

} // end namespace

} //end namespace
