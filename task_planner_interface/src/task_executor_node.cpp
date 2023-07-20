
#include <ros/ros.h>
#include <task_planner_interface/task_executor.h>

int main(int argc, char **argv)
{
  std::string action_name = "/task_execute";

  ros::init(argc, argv, "executor");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string topic_request_from_planner;
  if (!pnh.getParam("topic_request_from_planner",topic_request_from_planner))
  {
    ROS_ERROR("topic_request_from_planner not defined");
    return 0;
  }

  std::string group_name="";
  if (!pnh.getParam("group_name",group_name))
  {
    ROS_ERROR_STREAM(pnh.getNamespace() << ": group_name not defined");
    return 0;
  }

  std::string home_position="home";
  if (!pnh.getParam("home_position",home_position))
  {
    ROS_ERROR_STREAM(pnh.getNamespace() << ": home_position not defined");
  }

  std::string retry_position=home_position;
  if (!pnh.getParam("retry_position",retry_position))
  {
    ROS_ERROR_STREAM(pnh.getNamespace() << ": retry_position not defined");
  }

  bool go_home_after_execution = false;
  if (!nh.getParam("go_home_after_execution",go_home_after_execution))
  {
    ROS_ERROR("go_home_after_execution not defined. Default: false");
  }

  taskPlannerInterface::TaskExecutor task_executor(nh,
                                                   pnh,
                                                   topic_request_from_planner+action_name,
                                                   group_name,
                                                   home_position,
                                                   retry_position,
                                                   go_home_after_execution,
                                                   topic_request_from_planner + "/reset_agent_state");
  ros::spin();
  return 0;

}


