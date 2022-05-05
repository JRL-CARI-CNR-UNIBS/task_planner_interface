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
  ros::init(argc, argv, "dispatcher_single");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  bool do_reset=false;
  if (!pnh.getParam("reset",do_reset))
    ROS_ERROR("reset not defined. default:false");

  std::string topic_request_from_planner;
  if (!pnh.getParam("topic_request_from_planner",topic_request_from_planner))
  {
    ROS_ERROR("topic_request_from_planner not defined");
    return 0;
  }
  std::string topic_feedback_to_planner;
  if (!pnh.getParam("topic_feedback_to_planner",topic_feedback_to_planner))
  {
    ROS_ERROR("topic_motion_feedback not defined");
    return 0;
  }
  std::string plan_name;
  if (!pnh.getParam("plan_name",plan_name))
  {
    ROS_ERROR("plan_name not defined. default: robot_plan");
    plan_name="robot_plan";
  }

  ros::Publisher command_pub=nh.advertise<task_planner_interface_msgs::MotionTaskExecutionRequestArray>(topic_request_from_planner,1);

  ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback> feedback_notif(nh,topic_feedback_to_planner,1);
  feedback_notif.setAdvancedCallback(&newCommandCallback);
  if (!feedback_notif.waitForANewData(ros::Duration(1)))
  {
    ROS_ERROR_STREAM("timeout: no new messages from topic " << topic_feedback_to_planner);
//    return 0;
  }

  task_planner_interface_msgs::MotionTaskExecutionRequestArrayPtr current_command_msg(new task_planner_interface_msgs::MotionTaskExecutionRequestArray());

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
  }
  else
  {
    recipes.push_back(recipe);
    starting_el=0;
  }

  /* Sort recipes and start from element "starting_el" */
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

  unsigned int n_tasks = 3;

//  for (unsigned int idx=0;idx<recipes.size();idx++)
//  {
//    int r = idx + rand() % (recipes.size() - idx);
//    auto temp=recipes.at(idx);
//    recipes.at(idx)=recipes.at(r);
//    recipes.at(r)=temp;
//  }
//  for (auto it = recipes.begin(); it != recipes.end(); ++it)
//    std::cout << *it << std::endl;

  for (auto it = recipes.begin(); it != recipes.end(); ++it)
  {
    int fail_counter=0;
    if (it!=recipes.begin() && do_reset)
    {
        resetClient.call(trigger_srv);
        resetBoxClient.call(bool_srv);
        resetOutboundClient.call(bool_srv);
        addObjsClient.call(bool_srv);
    }

    ROS_INFO_STREAM("Executing recipe:" << *it);
    YAML::Node config = YAML::LoadFile(*it);
    const YAML::Node& robot_plan_node = config[plan_name];
    for (YAML::const_iterator it = robot_plan_node.begin(); it != robot_plan_node.end(); ++it)
    {
        const YAML::Node& robot_plan = *it;
        current_command_msg->cmd_id=robot_plan["cmd_id"].as<int>();
        current_command_msg->tasks.resize(n_tasks);

        YAML::const_iterator it2=it;
        unsigned int i_task=0;
        while (i_task<n_tasks && it2 != robot_plan_node.end())
        {
            const YAML::Node& robot_plan2 = *it2;
            current_command_msg->tasks.at(i_task).task_id=robot_plan2["task"].as<std::basic_string<char>>();
            ++it2;
            i_task++;
        }
        if (i_task<n_tasks)
          current_command_msg->tasks.resize(i_task);

        command_pub.publish(current_command_msg);

        std::cout << "command sent: " << current_command_msg->cmd_id << "\t"
                  << current_command_msg->tasks.at(0).task_id << "\n";

        ros::Duration timeout(300);
        ros::Rate loopRate(100);
        ros::Time init_time = ros::Time::now();
        while((ros::Time::now()-init_time)<timeout)
        {
          if (feedback_notif.isANewDataAvailable())
          {
            if (feedback_notif.getData().cmd_id==current_command_msg->cmd_id)
            {
              break;
            }
          }
          loopRate.sleep();
          ros::spinOnce();
        }
        if ((ros::Time::now()-init_time)>=timeout)
        {
          ROS_ERROR_STREAM("timeout: no feedback from topic " << topic_feedback_to_planner << ". Abort plan.");
          return 0;
        }
        else if (feedback_notif.getData().result == SUCCESS)
        {
          fail_counter=0;
          std::cout << "feedback received: SUCCESS. \n\n";
        }
        else
        {
          std::cout << "feedback received: FAILURE. Continueing...\n\n";
//          std::cout << "feedback received: FAILURE. Continue? (y/N).\n\n";
//          char input;
//          std::cin >> input;
//          if (input!='y')
//            return 0;

//            fail_counter++;
//            if (fail_counter>=3)
//            {
//              ROS_FATAL("aborting current recipe because of two consecutive failures.");
//              break;
//            }
        }
    }
    ROS_ERROR("RECIPE COMPLETED.");

  }
  ROS_INFO("all plans completed. Exit normally.");
  return 0;
}


