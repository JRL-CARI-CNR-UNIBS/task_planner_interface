#include <task_planner_interface/generic_skill.h>


namespace taskPlannerInterface
{

namespace skills {

#define SUCCESS 1
#define FAILURE 0

GenericSkill::GenericSkill()
{
  m_outcome=0;
  m_duration_real=0.0;
  m_duration_planned=0.0;
  m_planning_time=0.0;
  m_path_length=0.0;

  m_skill_properties_client = m_nh.serviceClient <task_planner_interface_msgs::BasicSkill>("mongo_handler/get_task_properties");
}

GenericSkill::GenericSkill(AgentStatusPtr agent_status_ptr)
{
  m_outcome=0;
  m_duration_real=0.0;
  m_duration_planned=0.0;
  m_planning_time=0.0;
  m_path_length=0.0;
  m_agent_status=agent_status_ptr;

  m_skill_properties_client = m_nh.serviceClient <task_planner_interface_msgs::BasicSkill>("mongo_handler/get_task_properties");
}

GenericSkill::GenericSkill(const ros::NodeHandle& nh,AgentStatusPtr agent_status_ptr):
    m_nh(nh)
{
  m_outcome=0;
  m_duration_real=0.0;
  m_duration_planned=0.0;
  m_planning_time=0.0;
  m_path_length=0.0;
  m_agent_status=agent_status_ptr;

  m_skill_properties_client = m_nh.serviceClient <task_planner_interface_msgs::BasicSkill>("mongo_handler/get_task_properties");
}

bool GenericSkill::execute()
{
  std::cout << "invoked genericSkill execute \n";
  return false;
}

void GenericSkill::init(const std::string& group_name)
{
  std::cout << "invoked genericSkill init (group_name: " << group_name << ")\n";
}

bool GenericSkill::setPropertiesFromBSON(bsoncxx::stdx::optional<bsoncxx::document::value>& bson_doc)
{
  bsoncxx::document::element ele_name{bson_doc->view()["name"]};
  m_id=ele_name.get_utf8().value.to_string();
  bsoncxx::document::element ele_type{bson_doc->view()["type"]};
  m_type=ele_type.get_utf8().value.to_string();
  try
  {
    bsoncxx::document::element ele_job{bson_doc->view()["job_name"]};
    m_job_name=ele_job.get_utf8().value.to_string();
  }
  catch (...)
  {
    m_job_name="";
  }




//  bsoncxx::document::element ele_duration{bson_doc->view()["duration_expected"]};
//  m_duration_expected=ele_duration.get_double().value;
//  bsoncxx::document::element ele_risklevel{bson_doc->view()["static_risk"]};
//  m_static_risk=ele_risklevel.get_double().value;
  return true;
}

bool GenericSkill::setPropertiesFromService(task_planner_interface_msgs::BasicSkill task_properties)
{

  m_id = task_properties.response.name;
  m_type = task_properties.response.type;
  if(task_properties.response.job_name.empty())
  {
      m_job_name = "" ;
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

bool GenericSkill::setPropertiesFromService(std::string name, std::string type)
{
    m_id = name;
    m_type = type;

    m_job_name = "";
  return true;
}

void GenericSkill::print() const
{
  std::cout << "Print skill:\n\t name: " << m_id
            << "\n\t type: " << m_type
            << "\n";
}

bool GenericSkill::setConcurrentTasksFromMsg(const task_planner_interface_msgs::MotionTaskExecutionRequestArray& msg)
{
  m_concurrent_tasks=msg.tasks.at(0).human_tasks;
  return true;
}

bsoncxx::document::value GenericSkill::getResultsAsBSON()
{
  auto builder = bsoncxx::builder::stream::document{};
  bsoncxx::document::value bson_doc = builder
    << "name" << getName()
    << "type" << getType()
    << "outcome" << m_outcome
    << "duration_planned" << m_duration_planned
    << "duration_real" << m_duration_real
    << "planning_time" << m_planning_time
    << "path_length" << m_path_length
    << "date" <<bsoncxx::types::b_date(std::chrono::system_clock::now())
  << bsoncxx::builder::stream::finalize;
  return bson_doc;
}

/* Return a TaskExecuteResult message containing the task execution info
 * @return taskResult
 * */
task_planner_interface_msgs::TaskExecuteResult GenericSkill::getResultsAsTaskExecuteResult(const std::string& group_name)
{
    task_planner_interface_msgs::TaskExecuteResult task_result;
    task_result.type = getType();
    task_result.agent = group_name;
    task_result.outcome = m_outcome;
    task_result.duration_planned = m_duration_planned;
    task_result.duration_real = m_duration_real;
    task_result.planning_time = m_planning_time;
    task_result.path_length = m_path_length;
    ROS_ERROR("Tipo: %s \n Outcome: %d \n Duration planned: %f",task_result.type.c_str() ,task_result.outcome,task_result.duration_planned);
    return task_result;
}

void GenericSkill::setAgentStatus(AgentStatusPtr agent_status_ptr){ m_agent_status=agent_status_ptr; }

void GenericSkill::getResults(int& outcome, double& duration_real, double& duration_planned, double& planning_time, double& path_length) const
{
  outcome=m_outcome;
  duration_real=m_duration_real;
  duration_planned=m_duration_planned;
  planning_time=m_planning_time;
  path_length=m_path_length;
}

} //end namespace

} //end namespace
