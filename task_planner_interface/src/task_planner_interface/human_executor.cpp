
#include <task_planner_interface/human_executor.h>

#define SUCCESS 1
#define FAILURE 0

namespace taskPlannerInterface
{

    HumanExecutor::HumanExecutor(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& pnh,
                                 const std::string action_name,
                                 const std::string group_name):
            m_nh(nh),
            m_pnh(pnh),
            m_action_name(action_name),
            m_group_name(group_name),
            m_as(nh, action_name, boost::bind(&HumanExecutor::executeTask, this, _1), false)
            {
              m_as.start();

              /* Server clients for mongo*/

              m_skill_type_client = m_nh.serviceClient <task_planner_interface_msgs::TaskType>("mongo_handler/check_task_type");
              m_skill_properties_client = m_nh.serviceClient <task_planner_interface_msgs::BasicSkill>("mongo_handler/get_task_properties_human");

              /*Publishers for hmi interaction*/
              m_pub_task_request = m_nh.advertise<task_planner_interface_msgs::MotionTaskExecutionRequestArray>("hmi/task_execution_request", 10);
              m_sub_task_result.reset(new ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback> (m_nh, "hmi/task_execution_result",10));

              ros::Duration m_t_total;


            }

    bool HumanExecutor::setPropertiesFromService(std::string name,std::string type,std::string& description)
    {
        task_planner_interface_msgs::BasicSkill srv;
        srv.request.name = name;
        if(m_skill_properties_client.call(srv))
        {
            if(!srv.response.error)   // If no error
            {
                ROS_INFO("Task properties retrieved correctly");
                description = srv.response.description;
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
    bool HumanExecutor::checkSkillType(const std::string name, std::string& skill_type)
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

    void HumanExecutor::executeTask(const task_planner_interface_msgs::TaskExecuteGoalConstPtr& goal)
    {
        ROS_INFO("Action server received task: %s",goal->name.c_str());
        std::string skill_name = goal->name;
        std::string skill_type;

        ros::Time t_start = ros::Time::now();
        ros::Rate r(10);

        taskPlannerInterface::skills::GenericSkillPtr current_skill;
        int current_result = FAILURE;

        if(checkSkillType(skill_name,skill_type))
        {
            ROS_INFO("Skill received");
            std::string skill_description;
            if(setPropertiesFromService(skill_name,skill_type,skill_description))
            {
              task_planner_interface_msgs::MotionTaskExecutionRequestArray human_command_msg;
              human_command_msg.tasks.resize(1);
              human_command_msg.tasks[0].task_id = skill_name;
              human_command_msg.tasks[0].task_description = skill_description;

              m_pub_task_request.publish(human_command_msg);

              ros::Time t_send = ros::Time::now();
              ROS_INFO("cmd sent. time = %f", t_send.toSec() );

              ROS_WARN("Waiting skill result...");

              bool fdk_received = false;
              task_planner_interface_msgs::MotionTaskExecutionFeedback skill_feedback;
              while (fdk_received==false)
              {
                if (m_as.isPreemptRequested() || !ros::ok())
                {
                    ROS_ERROR("%s: Preempted", m_action_name.c_str());
                    // set the action state to preempted
                    m_as.setPreempted();
                }

                if (m_sub_task_result->isANewDataAvailable())
                {
                   skill_feedback = m_sub_task_result->getData();
                   ros::Time t_msg;
                   m_sub_task_result->getMsgReceivedTime()->get(t_msg);
                   //ROS_INFO("new msg. time = %f", t_msg.toSec() );


                   if ((t_msg - t_send).toSec()>0.0)
                     fdk_received = true;
                }
                r.sleep();
              }

              ROS_INFO("Human skill result received");



              task_planner_interface_msgs::TaskExecuteResult m_result;
              m_result.type = skill_type;
              m_result.agent = m_group_name;
              m_result.outcome = SUCCESS;
              m_result.duration_real = (ros::Time::now()-t_start).toSec();

              ROS_INFO("Execution finished: %s", skill_name.c_str());
              m_as.setSucceeded(m_result);

            }
            else
            {
                ROS_FATAL_STREAM("Skill properties of:" << skill_type << " not retrieved correctly. Abort plan.");
                m_as.setAborted();
            }
        }
        else
        {
            ROS_FATAL_STREAM("Skill type:" << skill_type << " not defined. Abort plan.");
            m_as.setAborted();
        }




    }

} //end namespace
