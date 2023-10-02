#include <task_planner_interface/task_executor.h>

#define SUCCESS 1
#define FAILURE 0

namespace taskPlannerInterface
{

    TaskExecutor::TaskExecutor(const ros::NodeHandle& nh,
                               const ros::NodeHandle& pnh,
                               const std::string action_name,
                               const std::string group_name,
                               const std::string home_position,
                               const std::string retry_position,
                               const bool go_home_after_execution,
                               const std::string reset_agent_srv_name):
            m_nh(nh),
            m_pnh(pnh),
            m_action_name(action_name),
            m_as(nh, action_name, boost::bind(&TaskExecutor::executeTask, this, _1), false),
            m_group_name(group_name),
            m_home_position(home_position),
            m_retry_position(retry_position),
            m_go_home_after_execution(go_home_after_execution)
            {
              m_as.start();

              /* Server clients for mongo*/

              m_skill_type_client = m_nh.serviceClient <task_planner_interface_msgs::TaskType>("mongo_handler/check_task_type");

              /* Define AgentStatus */

              m_agent_status = std::make_shared<taskPlannerInterface::AgentStatus>();

              /* Changes Here */
              std::map<std::string, std::string> additional_pick_skill_properties;
              std::map<std::string, std::string> additional_place_skill_properties;
              taskPlannerInterface::skills::PickSkillPtr pick_skill;
              taskPlannerInterface::skills::PlaceSkillPtr place_skill;
              taskPlannerInterface::skills::PickPlaceSkillPtr pick_place_skill;

              if (getAdditionalProperties("pick_execution_properties", additional_pick_skill_properties)) {
                  pick_skill = std::make_shared<taskPlannerInterface::skills::PickSkill>(m_agent_status, additional_pick_skill_properties);
              } else {
                  pick_skill = std::make_shared<taskPlannerInterface::skills::PickSkill>(m_agent_status);
              }

              if (getAdditionalProperties("place_execution_properties", additional_place_skill_properties)) {
                  place_skill = std::make_shared<taskPlannerInterface::skills::PlaceSkill>(m_agent_status, additional_place_skill_properties);
              } else {
                  place_skill = std::make_shared<taskPlannerInterface::skills::PlaceSkill>(m_agent_status);
              }

              if (getAdditionalProperties("pickplace_execution_properties/pick", additional_pick_skill_properties) &&
                  getAdditionalProperties("pickplace_execution_properties/place", additional_place_skill_properties)) {
                  pick_place_skill = std::make_shared<taskPlannerInterface::skills::PickPlaceSkill>(m_agent_status, additional_pick_skill_properties, additional_place_skill_properties);
              } else {
                  pick_place_skill = std::make_shared<taskPlannerInterface::skills::PickPlaceSkill>(m_agent_status);
              }


              /* Changes Stop Here*/

              //Single skills
              /*
              taskPlannerInterface::skills::PickSkillPtr pick_skill(new taskPlannerInterface::skills::PickSkill(m_agent_status));
              taskPlannerInterface::skills::PlaceSkillPtr place_skill(new taskPlannerInterface::skills::PlaceSkill(m_agent_status));
              taskPlannerInterface::skills::PickPlaceSkillPtr pick_place_skill(new taskPlannerInterface::skills::PickPlaceSkill(m_agent_status));
              */
              taskPlannerInterface::skills::GoToSkillPtr goto_skill(new taskPlannerInterface::skills::GoToSkill(m_agent_status));

              //Fill map of skills (class attribute)
              m_skills["pick"] = pick_skill;
              m_skills["place"] = place_skill;
              m_skills["pickplace"] = pick_place_skill;
              m_skills["goto"] = goto_skill;

              /* Define a goTo skill for moving home */
              ROS_INFO("Waiting goto skill ...");

              //m_basic_goto_skill = std::make_shared<taskPlannerInterface::skills::GoToSkill>(m_agent_status);

              m_basic_goto_skill.reset(new taskPlannerInterface::skills::GoToSkill(m_agent_status));

              m_basic_goto_skill->init(m_group_name);

              m_reset_agent_state_srv  = m_nh.advertiseService(reset_agent_srv_name, &TaskExecutor::resetAgentState, this);

              ROS_INFO("Goto skill ok");

              ros::Duration m_t_total;


    }
    bool TaskExecutor::resetAgentState(std_srvs::Trigger::Request&  req,
                                       std_srvs::Trigger::Response& res)
    {
      m_agent_status->resetAgentStatus();
      res.success = true;
      res.message = "";
      ROS_INFO_STREAM("Agent status of: " << m_group_name << " resetted");
      return true;
    }

    bool TaskExecutor::checkSkillType(const std::string name, std::string& skill_type)
    {
        task_planner_interface_msgs::TaskType srv;
        srv.request.name = name;

        if(m_skill_type_client.call(srv))
        {
            if(srv.response.exist)
            {
                ROS_INFO("Task type: %s", srv.response.type.c_str());
                skill_type=srv.response.type;
                return true;
            }
            else
            {
                ROS_ERROR("Task type does not exist");

            }

        }
        else
        {
            ROS_ERROR("Failed to call service check_type");
        }
        return false;   // Both for service fail or task doesn't exist
    }

    void TaskExecutor::executeTask(const task_planner_interface_msgs::TaskExecuteGoalConstPtr& goal)
    {
        ROS_INFO("Action server received task: %s",goal->name.c_str());
        std::string skill_name = goal->name;
        std::string skill_type;

        ros::Time t_start = ros::Time::now();
        ros::Rate r(100);

        taskPlannerInterface::skills::GenericSkillPtr current_skill;
        int current_result = FAILURE;

        if(checkSkillType(skill_name,skill_type))
        {
            ROS_INFO("Skill received");
            auto it = m_skills.find(skill_type);

            if(it!=m_skills.end())
            {
              ros::Time t_now = ros::Time::now();

              /* Set skill properties calling service */
              it->second->setPropertiesFromService(skill_name,skill_type);
              ROS_INFO("Time set properties: %f", (ros::Time::now()-t_now).toSec());

              /* Initialize skill*/
              it->second->init(m_group_name);

              ROS_INFO("Time init+set: %f", (ros::Time::now()-t_now).toSec());

              current_skill = it->second;

              bool is_preempted = false;
              for (unsigned int i_try=0;i_try<5;i_try++)
              {
                ROS_INFO("Trial %d",i_try+1);
                current_skill->execute();
                current_result=current_skill->getOutcome();

                /*Check if action is preempted*/
                is_preempted=m_as.isPreemptRequested();

                if (current_result==SUCCESS && !is_preempted)
                  break;
                else
                {
                    ROS_WARN_STREAM(m_pnh.getNamespace() << ": skill execution failed. Retrying from another position...");
                    if (!m_retry_position.empty())
                    {

                      ROS_INFO("Go to: %s", m_retry_position.c_str());
                      m_basic_goto_skill->sendDirectLocation(m_retry_position);
                    }
                    ros::Duration(2.5).sleep();
                 }

                // Check that preempt has not been requested by the client
                if (is_preempted || !ros::ok())
                {
                  ROS_ERROR("%s: Preempted", m_action_name.c_str());
                  // set the action state to preempted
                  m_as.setPreempted();
                  break;
                }
              }

              if(!is_preempted) /* If the task is not set to preempted */
              {
                  //Verifica non preempted (goalCanceled) altrimenti segue...
                  if (current_result==FAILURE)
                  {
                      ROS_ERROR_STREAM(m_pnh.getNamespace() << ". skill failed. Time to move on... ");
                      // current_result = SUCCESS; // uncomment if you want to continue anyways
                      ros::Duration(1.0).sleep();
                  }
                  if (m_go_home_after_execution && !m_home_position.empty())
                  {
                      ROS_INFO("Home position at skill-end");
                      m_basic_goto_skill->sendDirectLocation(m_home_position);
                  }

                  /* Send action result to service to action client */
                  task_planner_interface_msgs::TaskExecuteResult m_result = current_skill->getResultsAsTaskExecuteResult(m_group_name);
                  ROS_INFO("Execution finished: %s", skill_name.c_str());
                  m_as.setSucceeded(m_result);
              }


            }
            else
            {
              /*
               * ??
                */
              ROS_FATAL_STREAM("Skill type:" << skill_type << " not defined. Abort plan.");
              m_as.setAborted();
            }
        }
        else
        {
             ROS_FATAL_STREAM("Skill not present in task properties");
             m_as.setAborted();
            /*
             *  Cosa fare in caso non sia recuperata la skill type
             */
        }


    }
    bool TaskExecutor::getAdditionalProperties(const std::string& skill_name,
                                               std::map<std::string, std::string>& additional_properties)
    {
      additional_properties.clear();
      if(m_nh.hasParam(skill_name))
      {
        XmlRpc::XmlRpcValue additional_properties_tmp;
        m_nh.getParam(skill_name, additional_properties_tmp);
        if(additional_properties_tmp.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
          ROS_ERROR("Param: %s, not well formatted, must be struct", skill_name);
          return false;
        }

        if(not(additional_properties_tmp.hasMember("tool_id") &&
               additional_properties_tmp.hasMember("pre_exec_id") &&
               additional_properties_tmp.hasMember("exec_id")))
        {
          ROS_ERROR("Missing correct additional properties to pick_skills: tool_id, pre_exec_id, exec_id");
          return false;
        }
        if (not(additional_properties_tmp["tool_id"].getType() == XmlRpc::XmlRpcValue::TypeString &&
                additional_properties_tmp["pre_exec_id"].getType() == XmlRpc::XmlRpcValue::TypeString &&
                additional_properties_tmp["exec_id"].getType() == XmlRpc::XmlRpcValue::TypeString))
        {
          ROS_ERROR("Additional properties are not string");
          return false;
        }
          additional_properties["tool_id" ]  = static_cast<std::string>(additional_properties_tmp["tool_id"]);
          additional_properties["pre_exec_id" ]  = static_cast<std::string>(additional_properties_tmp["pre_exec_id"]);
          additional_properties["exec_id" ]  = static_cast<std::string>(additional_properties_tmp["exec_id"]);
          return true;
      }
      return false;
    }

} //end namespace
