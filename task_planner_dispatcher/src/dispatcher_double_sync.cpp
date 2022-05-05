#include <ros/ros.h>
#include <task_planner_interface_msgs/MotionTaskExecutionFeedback.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequest.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequestArray.h>
#include <subscription_notifier/subscription_notifier.h>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
//#include <string>

#define SUCCESS 1
#define FAILURE 0

void newCommandCallback(const task_planner_interface_msgs::MotionTaskExecutionFeedbackConstPtr& msg){}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dispatcher_double_sync");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  bool do_reset=false;
  if (!pnh.getParam("reset",do_reset))
    ROS_ERROR("reset not defined. default:false");

  std::string human_request_from_planner;
  if (!pnh.getParam("human_request_from_planner",human_request_from_planner))
  {
    ROS_ERROR("human_request_from_planner not defined");
    return 0;
  }
  std::string robot_request_from_planner;
  if (!pnh.getParam("robot_request_from_planner",robot_request_from_planner))
  {
    ROS_ERROR("robot_request_from_planner not defined");
    return 0;
  }

  std::string human_feedback_to_planner;
  if (!pnh.getParam("human_feedback_to_planner",human_feedback_to_planner))
  {
    ROS_ERROR("human_feedback_to_planner not defined");
    return 0;
  }
  std::string robot_feedback_to_planner;
  if (!pnh.getParam("robot_feedback_to_planner",robot_feedback_to_planner))
  {
    ROS_ERROR("robot_feedback_to_planner not defined");
    return 0;
  }

  bool autosync=false;
  if (!nh.getParam("autosync",autosync))
    ROS_ERROR("autosync disabled by default");

  ros::Publisher robot_command_pub=nh.advertise<task_planner_interface_msgs::MotionTaskExecutionRequestArray>(robot_request_from_planner,1);
  ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback> robot_feedback_notif(nh,robot_feedback_to_planner,1);
  robot_feedback_notif.setAdvancedCallback(&newCommandCallback);
  if (!robot_feedback_notif.waitForANewData(ros::Duration(1)))
    ROS_ERROR_STREAM("timeout: no new messages from topic " << robot_feedback_to_planner);

  ros::Publisher human_command_pub=nh.advertise<task_planner_interface_msgs::MotionTaskExecutionRequestArray>(human_request_from_planner,1);
  ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback> human_feedback_notif(nh,human_feedback_to_planner,1);
  human_feedback_notif.setAdvancedCallback(&newCommandCallback);
  if (!human_feedback_notif.waitForANewData(ros::Duration(1)))
    ROS_ERROR_STREAM("timeout: no new messages from topic " << human_feedback_to_planner);

  task_planner_interface_msgs::MotionTaskExecutionRequestArrayPtr robot_command_msg(new task_planner_interface_msgs::MotionTaskExecutionRequestArray());
  task_planner_interface_msgs::MotionTaskExecutionRequestArrayPtr human_command_msg(new task_planner_interface_msgs::MotionTaskExecutionRequestArray());
  task_planner_interface_msgs::MotionTaskExecutionRequestArrayPtr sync_command_msg(new task_planner_interface_msgs::MotionTaskExecutionRequestArray());

  sync_command_msg->cmd_id=-1;
  sync_command_msg->tasks.resize(1);
  sync_command_msg->tasks.at(0).task_id="syncronization";

  std::string recipe;
  std::vector<std::string> recipes;
  if (!nh.getParam("recipe",recipe))
  {
    ROS_ERROR("recipe not defined. Checking for folder path");
    std::string recipe_path;
    if (!nh.getParam("recipe_path",recipe_path))
    {
      ROS_ERROR("recipe path not defined");
      return 0;
    }
    else
    {
      for (const auto & entry : boost::filesystem::directory_iterator(recipe_path))
      {
        std::string temp{entry.path().string()};
        recipes.push_back(temp);
      }
    }
  }
  else
    recipes.push_back(recipe);

  /* Sort recipes and start from element "starting_el" */
  int starting_el=0;
  if (!nh.getParam("starting_recipe_number",starting_el))
  {
    ROS_ERROR("starting recipe not defined. Default: 0");
  }
  std::sort(recipes.begin(), recipes.end());
  std::vector<std::string> temp_vec;
  for (unsigned int el=starting_el;el<recipes.size();el++)
    temp_vec.push_back(recipes.at(el));
  for (unsigned int el=0;el<starting_el;el++)
    temp_vec.push_back(recipes.at(el));
  recipes=temp_vec;
  std::cout << "List of recipes to execute:" << std::endl;
  for (auto it = recipes.begin(); it != recipes.end(); ++it)
    std::cout << *it << std::endl;

  /* Services to reset scene */

  std_srvs::Trigger trigger_srv;
  std_srvs::SetBool bool_srv;
  bool_srv.request.data = true;

  ros::ServiceClient resetClient = nh.serviceClient<std_srvs::Trigger>("/reset_scene");
  ros::ServiceClient resetBoxClient = nh.serviceClient<std_srvs::SetBool>("/inbound/reset_box");
  ros::ServiceClient resetOutboundClient = nh.serviceClient<std_srvs::SetBool>("/outbound/reset");
  ros::ServiceClient addObjsClient = nh.serviceClient<std_srvs::SetBool>("/inbound/add_objects");

  if (do_reset)
  {
    ROS_INFO("waiting for existence of services...");
    resetClient.waitForExistence();
    resetBoxClient.waitForExistence();
    resetOutboundClient.waitForExistence();
    ROS_INFO("services exist.");
  }

  std::string robot_plan_name="robot_plan";
  std::string human_plan_name="human_plan";
  unsigned int n_tasks = 3;

  for (auto it_recipe = recipes.begin(); it_recipe != recipes.end(); ++it_recipe)
  {
    int fail_counter=0;
    if (it_recipe!=recipes.begin() && do_reset)
    {
      ROS_INFO("resetting scene...");
      resetBoxClient.call(bool_srv);
      resetOutboundClient.call(bool_srv);
      resetClient.call(trigger_srv);
      addObjsClient.call(bool_srv);
      ROS_INFO("scene resetted...");
    }

    ROS_INFO_STREAM("Executing recipe:" << *it_recipe);
    YAML::Node config = YAML::LoadFile(*it_recipe);
    const YAML::Node& robot_plan_node = config[robot_plan_name];
    const YAML::Node& human_plan_node = config[human_plan_name];
    if (robot_plan_node.size() != human_plan_node.size() )
    {
      ROS_FATAL("HUMAN PLAN AND ROBOT PLAN HAVE DIFFERENT SIZES. ABORT.");
      return 0;
    }

    YAML::const_iterator it_human = human_plan_node.begin();
    for (YAML::const_iterator it_robot = robot_plan_node.begin(); it_robot != robot_plan_node.end(); ++it_robot)
    {
        const YAML::Node& robot_plan = *it_robot;
        const YAML::Node& human_plan = *it_human;

        /* Fill robot msg */
        robot_command_msg->cmd_id=robot_plan["cmd_id"].as<int>();
        robot_command_msg->tasks.resize(n_tasks);
        YAML::const_iterator it_robot2=it_robot;
        unsigned int i_task=0;
        while (i_task<n_tasks && it_robot2 != robot_plan_node.end())
        {
            const YAML::Node& robot_plan2 = *it_robot2;
            robot_command_msg->tasks.at(i_task).task_id=robot_plan2["task"].as<std::basic_string<char>>();
            ++it_robot2;
            i_task++;
        }
        if (i_task<n_tasks)
        {
          robot_command_msg->tasks.resize(i_task);
          ROS_INFO("size robot_command_msg=%d", robot_command_msg->tasks.size());
        }

        /* Fill human msg */
        human_command_msg->cmd_id=human_plan["cmd_id"].as<int>();
        human_command_msg->tasks.resize(n_tasks);
        YAML::const_iterator it_human2=it_human;
        i_task=0;
        while (i_task<n_tasks && it_human2 != human_plan_node.end())
        {
            const YAML::Node& human_plan2 = *it_human2;
            human_command_msg->tasks.at(i_task).task_id=human_plan2["task"].as<std::basic_string<char>>();
            ++it_human2;
            i_task++;
        }
        if (i_task<n_tasks)
        {
          human_command_msg->tasks.resize(i_task);
          ROS_INFO("size human_command_msg=%d", human_command_msg->tasks.size());
        }
        /* Publish msgs */
        robot_command_pub.publish(robot_command_msg);
        human_command_pub.publish(human_command_msg);


        std::cout << "command sent to robot: " << robot_command_msg->cmd_id << "\t"
                  << robot_command_msg->tasks.at(0).task_id << "\n";
        std::cout << "command sent to human: " << human_command_msg->cmd_id << "\t"
                  << human_command_msg->tasks.at(0).task_id << "\n\n";

        ros::Duration timeout(120);
        ros::Rate loopRate(100);
        ros::Time init_time = ros::Time::now();
        int robot_outcome=FAILURE;
        int human_outcome=FAILURE;
        bool is_robot_done=false;
        bool is_human_done=false;
        while((ros::Time::now()-init_time)<timeout && (!is_robot_done || !is_human_done) )
        {
          if (robot_feedback_notif.isANewDataAvailable())
          {
            if (robot_feedback_notif.getData().cmd_id==robot_command_msg->cmd_id)
            {
              robot_outcome=robot_feedback_notif.getData().result;
              if (autosync==true)
              {
                ROS_INFO("Autosync: robot feedback received, moving to home");
                sync_command_msg->cmd_id=-robot_command_msg->cmd_id;
                robot_command_pub.publish(sync_command_msg);
              }
              else
                is_robot_done=true;
            }
            else if (robot_feedback_notif.getData().cmd_id==-robot_command_msg->cmd_id)
            {
              is_robot_done=true;
              robot_outcome *= robot_feedback_notif.getData().result;
            }
          }
          if (human_feedback_notif.isANewDataAvailable())
          {
            if (human_feedback_notif.getData().cmd_id==human_command_msg->cmd_id)
            {
              human_outcome=human_feedback_notif.getData().result;
              if (autosync==true)
              {
                ROS_INFO("Autosync: human feedback received, moving to home");
                sync_command_msg->cmd_id=-human_command_msg->cmd_id;
                human_command_pub.publish(sync_command_msg);
              }
              else
                is_human_done=true;
            }
            else if (human_feedback_notif.getData().cmd_id==-human_command_msg->cmd_id)
            {
              is_human_done=true;
              human_outcome *= human_feedback_notif.getData().result;
            }
          }
          loopRate.sleep();
          ros::spinOnce();
        }

        if ((ros::Time::now()-init_time)>=timeout)
        {
          if (is_robot_done==false)
            ROS_ERROR("timeout: no feedback from robot. Abort plan.");
          if (is_human_done==false)
            ROS_ERROR("timeout: no feedback from human. Abort plan.");
          break;
        }

        if (robot_outcome == SUCCESS && human_outcome == SUCCESS)
        {
          fail_counter=0;
          std::cout << "feedbacks received: SUCCESS. \n\n";
        }
        else
        {
          fail_counter++;
          std::cout << "feedbacks received: FAILURE -> ";
          if (robot_outcome == FAILURE)
            std::cout << "robot. Failure counter = " << fail_counter << std::endl;
          else
            std::cout << "human. Failure counter = " << fail_counter << std::endl;
        }
        if (fail_counter > 3)
        {
          ROS_FATAL("failed too many times (three in a row). Moving to the next recipe...");
          break;
        }
        ++it_human;
    }
    ROS_ERROR("RECIPE COMPLETED OR ABORTED.");
    ros::Duration(2.0).sleep();
  }
  ROS_INFO("all plans completed. Exit normally.");
  return 0;
}


