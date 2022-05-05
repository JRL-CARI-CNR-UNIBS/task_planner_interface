#include <task_planner_interface/go_to_skill.h>

namespace taskPlannerInterface
{

namespace skills {

#define SUCCESS 1
#define FAILURE 0

GoToSkill::GoToSkill(){}

bool GoToSkill::execute()
{

  manipulation_msgs::GoToGoal location;
  location.location_names.clear();
  location.location_names=m_goal_location;
  ROS_INFO_STREAM("Command " << m_id << ": move to locations:");
  for (auto it=m_goal_location.begin(); it!=m_goal_location.end(); ++it)
  {
    std::cout << "- " << it->data() << std::endl;
  }

  m_goto_ac->sendGoalAndWait(location);
  if (m_goto_ac->getResult()->result<0)
  {
    m_outcome = FAILURE;
    m_duration_real=0.0;
    m_duration_planned=0.0;
    m_planning_time=0.0;
    m_path_length=0.0;
    ROS_ERROR_STREAM("Command " << m_id << ": cannot move to desired location.");
  }
  else
  {
    m_outcome = SUCCESS;
    m_duration_real=m_goto_ac->getResult()->actual_duration.toSec();
    m_duration_planned=m_goto_ac->getResult()->expected_execution_duration.toSec();
    m_planning_time=m_goto_ac->getResult()->planning_duration.toSec();
    m_path_length=m_goto_ac->getResult()->path_length;
    ROS_INFO_STREAM("Command " << m_id << ": move ok");
  }
  return (bool)m_outcome;
}

void GoToSkill::init(const std::string& group_name)
{
  if (m_job_name.empty())
  {
    m_job_name="go_to";
  }
  m_outcome=FAILURE;
  m_goto_ac.reset(new actionlib::SimpleActionClient<manipulation_msgs::GoToAction>("/go_to_location_server/"+group_name+"/"+m_job_name));

  m_goto_ac->waitForServer();

  ROS_INFO("Initialized goto skill for movegroup %s", group_name.c_str());
}

bool GoToSkill::setPropertiesFromBSON(bsoncxx::stdx::optional<bsoncxx::document::value>& bson_doc){
  if(!GenericSkill::setPropertiesFromBSON(bson_doc))
    return false;

  m_goal_location.clear();

//  for (auto it = ele_goal.get_array().value.begin(); it != ele_goal.get_array().value.end(); ++it)
//  {
//    m_goal_location.push_back(it->data());
//  }

  bsoncxx::document::element ele_goal{bson_doc->view()["goal"]};
  bsoncxx::array::view ele_goal_view{ele_goal.get_array().value};
  for (bsoncxx::array::element subdocument : ele_goal_view){
      //std::cout<< subdocument["FunctionalArea"].get_utf8().value << std::endl;
      m_goal_location.push_back(subdocument.get_utf8().value.to_string());
  }


  //m_goal_location=ele_goal.get_utf8().value.to_string();

  try
  {
    bsoncxx::document::element ele_job{bson_doc->view()["job_name"]};
    m_job_name=ele_job.get_utf8().value.to_string();
  }
  catch (...)
  {
    m_job_name="go_to";
  }

  return true;
}

bool GoToSkill::setPropertiesFromService(task_planner_interface_msgs::BasicSkill task_properties)
{
  m_goal_location.clear();

  m_goal_location = task_properties.response.goal;

//  goal_array = task_properties.response.goal;

//  for(std::string single_goal : goal_array)
//  {
//      m_goal_location.push_back(single_goal);
//  }

  if(task_properties.response.job_name.empty())
  {
      m_job_name = "go_to";
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

bool GoToSkill::setPropertiesFromService(std::string name,std::string type)
{
  GenericSkill::setPropertiesFromService(name,type);
  m_goal_location.clear();

  task_planner_interface_msgs::BasicSkill srv;
  srv.request.name = name;

  if(m_skill_properties_client.call(srv))
  {
      if(!srv.response.error)   // If no error
      {
          ROS_INFO("Task properties retrieved correctly");
          m_goal_location = srv.response.goal;

          if(srv.response.job_name.empty())
          {
              m_job_name = "go_to";
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

void GoToSkill::setGoal(std::vector<std::string>& arg){ m_goal_location=arg; }

bool GoToSkill::sendDirectLocation(const std::string& goal_location)
{

  bool outcome = false; 
  manipulation_msgs::GoToGoal location;
  location.location_names.clear();
  location.location_names.push_back(goal_location);
  ROS_INFO_STREAM("Direct send: move to location " << goal_location);
  m_goto_ac->sendGoalAndWait(location,ros::Duration(60.0));


  if (m_goto_ac->getResult()->result<0)
  {
    outcome = false;
    ROS_ERROR("Direct send: cannot move to desired location.");
  }
  else
  {
    outcome = true;
    ROS_INFO("Direct send: move ok");
  }
  return outcome;
}

} // end namespace

} //end namespace
