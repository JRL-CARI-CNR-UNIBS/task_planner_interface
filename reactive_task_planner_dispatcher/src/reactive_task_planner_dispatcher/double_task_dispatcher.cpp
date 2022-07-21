
#include "reactive_task_planner_dispatcher/double_task_dispatcher.h"


DoubleTaskDispatcher::DoubleTaskDispatcher(const std::string &name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
{


}

void DoubleTaskDispatcher::publishTask(const ros::Publisher &pub,
                                       const std::string &task_name)
{
    task_planner_interface_msgs::MotionTaskExecutionRequestArray task_to_publish;
    task_planner_interface_msgs::MotionTaskExecutionRequest task;
    task.task_id = task_name;
    task_to_publish.tasks={task};
    pub.publish(task_to_publish);
    ros::spinOnce();
}


BT::NodeStatus DoubleTaskDispatcher::tick()
{
  ROS_INFO_STREAM("---------------------------");
  ROS_INFO_STREAM("DoubleTaskDispatcher ticked");

  /* Input from behavior scheme */

  getInputWithCheck("task_name_first_agent",task_name_first_agent_);
  getInputWithCheck("first_agent_name",first_agent_name_);
  ROS_INFO_STREAM("Task Name First Agent: " << task_name_first_agent_);
  ROS_INFO_STREAM("First Agent Name: " << first_agent_name_);

  getInputWithCheck("task_name_second_agent",task_name_second_agent_);
  getInputWithCheck("second_agent_name",second_agent_name_);
  ROS_INFO_STREAM("Task Name Second Agent: " << task_name_second_agent_);
  ROS_INFO_STREAM("Second Agent Name: " << second_agent_name_);

  std::map<std::string,std::string> agents_requests; //{first_agent_name_,second_agent_name_};
  agents_requests[first_agent_name_]=task_name_first_agent_;
  agents_requests[second_agent_name_]=task_name_second_agent_;

  std::map<std::string,bool> wait_agents;
  getInputWithCheck("wait_first_agent",wait_agents[first_agent_name_]);
  getInputWithCheck("wait_second_agent",wait_agents[second_agent_name_]);
  ROS_INFO_STREAM("Aspetta primo agenteeeeeee: " << wait_agents[first_agent_name_]);
  ROS_INFO_STREAM("Aspetta secondo agenteeeeeee: " << wait_agents[second_agent_name_]);

  std::string task_feedback_topic_name,task_request_topic_name;

  for(auto it = agents_requests.begin(); it!= agents_requests.end();it++)
  {
    if (!nh_.getParam("task_feedback_"+it->first,task_feedback_topic_name))
    {
      ROS_ERROR_STREAM("task_feedback_" << it->first << " not defined");
      throw BT::RuntimeError("Missing required parameter task_feedback_"+it->first);
    }
    if (!nh_.getParam("task_request_"+it->first,task_request_topic_name))
    {
      ROS_ERROR_STREAM("task_request_" << it->first <<" not defined");
      throw BT::RuntimeError("Missing required parameter task_request_"+it->first);
    }
    task_feedback_sub_[it->first].reset(new ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback> (nh_, task_feedback_topic_name,10));
    task_request_pub_[it->first] = nh_.advertise<task_planner_interface_msgs::MotionTaskExecutionRequestArray>(task_request_topic_name, 1000);

    /* Publish task request */
    publishTask(task_request_pub_[it->first],it->second);
  }

//  for(auto it = agents_requests.begin(); it!= agents_requests.end();it++)
//  {
//    /* Wait task response */
//    ROS_INFO_STREAM("Waiting task feedback...");
//    while(!task_feedback_sub_[it->first]->isANewDataAvailable() && ros::ok())
//    {
//      ros::Duration(0.1).sleep();
//      ros::spinOnce();
//    }
//    task_feedback_sub_[it->first]->getData();
//  }

//    DoubleTaskDispatcher::setOutput("exchange_info_out","pezzo 1");
//  /* If there is exchange info in input put it in output */
//  std::string exchange_info;
//  BT::Optional<std::string> msg = DoubleTaskDispatcher::getInput<std::string>("exchange_info");
//  if(msg)
//  {
//    DoubleTaskDispatcher::setOutput("exchange_info_out","agente due ciaooo");
//  }

//  ROS_INFO_STREAM("Task " << task_name_ << "executed.");
  return BT::NodeStatus::SUCCESS;
}

DoubleTaskDispatcher::~DoubleTaskDispatcher()
{
}
