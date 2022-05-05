
#include <ros/ros.h>
#include <task_planner_interface/human_executor.h>


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
  std::string group_name = "";
  if(!pnh.getParam("group_name",group_name))
  {
      ROS_ERROR_STREAM(pnh.getNamespace() << ": group_name not defined");
      return 0;
  }

  taskPlannerInterface::HumanExecutor human_executor(nh,pnh,topic_request_from_planner+action_name,group_name);
  ros::spin();
  return 0;

}


