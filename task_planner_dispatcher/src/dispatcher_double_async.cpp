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

bool myComparator(const std::string& s1, const std::string& s2){
  if(s1.size() == s2.size())
  {
    return s1 < s2;
  }
  else
  {
    return s1.size() < s2.size();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dispatcher_double_async");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
    ROS_ERROR("-----------------------------");
  bool do_reset=true;
//  if (!pnh.getParam("reset",do_reset))
//    ROS_ERROR("reset not defined. default:false");

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

//  std::string plan_name;
//  if (!pnh.getParam("plan_name",plan_name))
//  {
//    ROS_ERROR("plan_name not defined. default: robot_plan");
//    plan_name="robot_plan";
//  }

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

  int starting_el=0;
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
    if (!nh.getParam("starting_recipe_number",starting_el))
    {
      ROS_ERROR("starting recipe not defined. Default: 0");
    }
    else
    {
      ROS_ERROR_STREAM("Starting recipe from: "<<starting_el);
    }
  }
  else
  {
    recipes.push_back(recipe);
    starting_el=0;
  }

  /* Sort recipes and start from element "starting_el" */
  std::sort(recipes.begin(), recipes.end(),myComparator);
  std::vector<std::string> temp_vec;
  for (unsigned int el=starting_el;el<recipes.size();el++)
    temp_vec.push_back(recipes.at(el));
//  for (unsigned int el=0;el<starting_el;el++)
//    temp_vec.push_back(recipes.at(el));
  recipes=temp_vec;
  std::cout << "List of recipes to execute:" << std::endl;

  for (auto it = recipes.begin(); it != recipes.end(); ++it)
    std::cout << *it << std::endl;


  /* Services to reset scene */

  std_srvs::Trigger trigger_srv;
  std_srvs::SetBool bool_srv;
  bool_srv.request.data = true;

  ros::ServiceClient resetClient = nh.serviceClient<std_srvs::Trigger>("/reset_scene");
  ros::ServiceClient resetBoxClient = nh.serviceClient<std_srvs::SetBool>("/inbound_pick_server/remove_all_objects");
  ros::ServiceClient resetOutboundClient = nh.serviceClient<std_srvs::SetBool>("/outbound_place_server/outbound/reset_all_slot");
  ros::ServiceClient addObjsClient = nh.serviceClient<std_srvs::SetBool>("/inbound_pick_loader/add_objects");
  ROS_ERROR_STREAM("do_reset");  ROS_ERROR_STREAM(do_reset);
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
    bool aborting = false;
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
      ROS_WARN("human plan and robot plan have different size.");

    bool robot_ready=true;
    bool human_ready=true;

    bool robot_waiting_for_sync = false;
    bool human_waiting_for_sync = false;

    int robot_outcome=FAILURE;
    int human_outcome=FAILURE;
    ros::Duration timeout(240);
    ros::Rate loopRate(100);
    ros::Time human_init_time = ros::Time::now();
    ros::Time robot_init_time = ros::Time::now();

    YAML::const_iterator it_robot = robot_plan_node.begin();
    YAML::const_iterator it_human = human_plan_node.begin();
    while ( it_robot != robot_plan_node.end() || it_human != human_plan_node.end() )
    {

      if (robot_waiting_for_sync && human_waiting_for_sync)
      {
        robot_waiting_for_sync = false;
        human_waiting_for_sync = false;
      }

      const YAML::Node& robot_plan = *it_robot;
      const YAML::Node& human_plan = *it_human;
      if (robot_ready && it_robot != robot_plan_node.end() && !robot_waiting_for_sync)
      {
        /* Fill robot msg */
        robot_command_msg->cmd_id=robot_plan["cmd_id"].as<int>();
        robot_command_msg->tasks.resize(n_tasks);
        YAML::const_iterator it_robot2=it_robot;
        if (aborting)
        {
          YAML::const_iterator it_robot3=it_robot2;
          while (it_robot3!=robot_plan_node.end())
          {
            ++it_robot3;
            if (it_robot3!=robot_plan_node.end())
              it_robot2=it_robot3;
          }
        }
        unsigned int i_task=0;
        while (i_task<n_tasks && it_robot2 != robot_plan_node.end())
        {
            const YAML::Node& robot_plan2 = *it_robot2;
            robot_command_msg->tasks.at(i_task).task_id=robot_plan2["task"].as<std::basic_string<char>>();
            try
            {
              robot_command_msg->tasks.at(i_task).task_description=robot_plan2["description"].as<std::basic_string<char>>();
            }
            catch (...)
            {
              ROS_WARN_THROTTLE(30,"Description field missing");
            }

            ++it_robot2;
            i_task++;
        }
        if (i_task<n_tasks)
        {
          robot_command_msg->tasks.resize(i_task);
          ROS_INFO("size robot_command_msg=%d", robot_command_msg->tasks.size());
        }

        /* Publish msg */

        if (robot_command_msg->tasks.at(0).task_id.compare("wait")==0)
        {
          robot_waiting_for_sync = true;
        }
        else
        {
          robot_command_pub.publish(robot_command_msg);
          std::cout << "command sent to robot: " << robot_command_msg->cmd_id << "\t"
                    << robot_command_msg->tasks.at(0).task_id <<"\n";
          robot_init_time = ros::Time::now();
          robot_outcome=FAILURE;
          robot_ready=false;
        }
        ++it_robot;
      }

      if (human_ready && it_human != human_plan_node.end() && !human_waiting_for_sync)
      {

        /* Fill human msg */
        human_command_msg->cmd_id=human_plan["cmd_id"].as<int>();
        human_command_msg->tasks.resize(n_tasks);
        YAML::const_iterator it_human2=it_human;
        if (aborting)
        {
          YAML::const_iterator it_human3=it_human2;
          while (it_human3!=human_plan_node.end())
          {
            ++it_human3;
            if (it_human3!=human_plan_node.end())
              it_human2=it_human3;
          }
        }
        unsigned int  i_task=0;
        while (i_task<n_tasks && it_human2 != human_plan_node.end())
        {
            const YAML::Node& human_plan2 = *it_human2;
            human_command_msg->tasks.at(i_task).task_id=human_plan2["task"].as<std::basic_string<char>>();
            try
            {
              human_command_msg->tasks.at(i_task).task_description=human_plan2["description"].as<std::basic_string<char>>();
            }
            catch (...)
            {
              ROS_WARN_THROTTLE(30,"Description field missing");
            }

            ++it_human2;
            i_task++;
        }
        if (i_task<n_tasks)
        {
          human_command_msg->tasks.resize(i_task);
          ROS_INFO("size human_command_msg=%d", human_command_msg->tasks.size());
        }

        if (human_command_msg->tasks.at(0).task_id.compare("wait")==0)
        {
          human_waiting_for_sync = true;
        }
        else
        {
          /* Publish msg */
          human_command_pub.publish(human_command_msg);
          std::cout << "command sent to human: " << human_command_msg->cmd_id << "\t"
                    << human_command_msg->tasks.at(0).task_id <<"\n\n";
          human_init_time = ros::Time::now();
          human_outcome=FAILURE;
          human_ready=false;
        }
        ++it_human;
      }


      if (robot_feedback_notif.isANewDataAvailable())
      {
        if (robot_feedback_notif.getData().cmd_id==robot_command_msg->cmd_id)
        {
          robot_outcome=robot_feedback_notif.getData().result;
          ROS_INFO("robot feedback received.");
          if (autosync==true)
          {
            sync_command_msg->cmd_id=10000+robot_command_msg->cmd_id;
            robot_command_pub.publish(sync_command_msg);
          }
          else
            robot_ready=true;
        }
        else if (robot_feedback_notif.getData().cmd_id==10000+robot_command_msg->cmd_id) // sync command
        {
          robot_ready=true;
          robot_outcome *= robot_feedback_notif.getData().result;
        }
      }
      else if (ros::Time::now()-robot_init_time > timeout && !robot_ready)
      {
        ROS_ERROR("timeout: no feedback from robot. Abort plan.");
        robot_ready=true;
        break;
      }

      if (human_feedback_notif.isANewDataAvailable())
      {
        if (human_feedback_notif.getData().cmd_id==human_command_msg->cmd_id)
        {
          human_outcome=human_feedback_notif.getData().result;
          ROS_INFO("human feedback received.");
          if (autosync==true)
          {
            sync_command_msg->cmd_id=10000+human_command_msg->cmd_id;
            human_command_pub.publish(sync_command_msg);
          }
          else
            human_ready=true;
        }
        else if (human_feedback_notif.getData().cmd_id==10000+human_command_msg->cmd_id) // sync command
        {
          human_ready=true;
          human_outcome *= human_feedback_notif.getData().result;
        }
      }
      else if (ros::Time::now()-human_init_time > timeout && !human_ready)
      {
        ROS_ERROR("timeout: no feedback from human. Abort plan.");
        human_ready=true;
        break;
      }

      loopRate.sleep();
      ros::spinOnce();

      if (robot_ready && it_human != robot_plan_node.end() && !robot_waiting_for_sync)
      {
        if (robot_outcome==FAILURE)
        {
          fail_counter++;
          ROS_WARN("new robot failure received. Failure counter = %d", fail_counter);
        }
      }
      if (human_ready && it_human != human_plan_node.end() && !human_waiting_for_sync)
      {
        if (human_outcome==FAILURE)
        {
          fail_counter++;
          ROS_WARN("new human failure received. Failure counter = %d", fail_counter);
        }
      }

      if (fail_counter > 3)
      {
        ROS_FATAL_STREAM("failed too many times. Aborting recipe:" << *it_recipe);
        aborting = true;
        break;
      }
    }
    ROS_ERROR("RECIPE COMPLETED OR ABORTED.");
    ros::Duration(2.0).sleep();
  }
  ROS_INFO("all plans completed. Exit normally.");
  return 0;
}
