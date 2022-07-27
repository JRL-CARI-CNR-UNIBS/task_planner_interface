
#include "reactive_task_planner_dispatcher/double_task_dispatcher.h"


DoubleTaskDispatcher::DoubleTaskDispatcher(const std::string &name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
{
  /* Input from behavior scheme */

  getInputWithCheck("task_name_first_agent",task_name_first_agent_);
  getInputWithCheck("first_agent_name",first_agent_name_);


//  ROS_INFO_STREAM("Task Name First Agent: " << task_name_first_agent_);
//  ROS_INFO_STREAM("First Agent Name: " << first_agent_name_);

  getInputWithCheck("task_name_second_agent",task_name_second_agent_);
  getInputWithCheck("second_agent_name",second_agent_name_);
//  ROS_INFO_STREAM("Task Name Second Agent: " << task_name_second_agent_);
//  ROS_INFO_STREAM("Second Agent Name: " << second_agent_name_);




  getInputWithCheck("wait_first_agent",wait_agents_[first_agent_name_]);
  getInputWithCheck("wait_second_agent",wait_agents_[second_agent_name_]);
//  ROS_INFO_STREAM("Wait first agent: " << wait_agents_[first_agent_name_]);
//  ROS_INFO_STREAM("Wait second agent: " << wait_agents_[second_agent_name_]);

  std::string task_feedback_topic_name,task_request_topic_name;
  agents_requests_[first_agent_name_]=task_name_first_agent_;
  agents_requests_[second_agent_name_]=task_name_second_agent_;
  for(auto it = agents_requests_.begin(); it!= agents_requests_.end();it++)
  {
    /* Get nedded parameters */
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
  }


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
  std::string piece_first_agent_input, piece_second_agent_input, task_name_first_agent, task_name_second_agent;
  task_name_first_agent = task_name_first_agent_;
  task_name_second_agent = task_name_second_agent_;
  if(DoubleTaskDispatcher::getInput("piece_first_agent_input",piece_first_agent_input))
  {
    task_name_first_agent = task_name_first_agent_ + "-" + piece_first_agent_input + "-" + first_agent_name_;
    DoubleTaskDispatcher::setOutput("piece_first_agent_output",piece_first_agent_input);
  }
  if(DoubleTaskDispatcher::getInput("piece_second_agent_input",piece_second_agent_input))
  {
    task_name_second_agent = task_name_second_agent_ + "-" + piece_second_agent_input + "-" + second_agent_name_;
    DoubleTaskDispatcher::setOutput("piece_second_agent_output",piece_second_agent_input);
  }
  agents_requests_[first_agent_name_]=task_name_first_agent;
  agents_requests_[second_agent_name_]=task_name_second_agent;
  ROS_INFO_STREAM("Task Name First Agent: " << task_name_first_agent);
  ROS_INFO_STREAM("Task Name Second Agent: " << task_name_second_agent);

  for(auto it = agents_requests_.begin(); it!= agents_requests_.end();it++)
  {
    /* Reset get data */
    if(task_feedback_sub_[it->first]->isANewDataAvailable())
    {
      task_feedback_sub_[it->first]->getData();
    }
    /* Publish task request */
    publishTask(task_request_pub_[it->first],it->second);
  }

  while(ros::ok())
  {
    if(wait_agents_[first_agent_name_] && wait_agents_[second_agent_name_])
    {
      if(task_feedback_sub_[first_agent_name_]->isANewDataAvailable() && task_feedback_sub_[second_agent_name_]->isANewDataAvailable())
        break;
    }
    else if(wait_agents_[first_agent_name_] && !wait_agents_[second_agent_name_])
    {
      if(task_feedback_sub_[first_agent_name_]->isANewDataAvailable())
        break;
    }
    else if(!wait_agents_[first_agent_name_] && wait_agents_[second_agent_name_])
    {
      if(task_feedback_sub_[second_agent_name_]->isANewDataAvailable())
        break;
    }
    else
    {
      break;
    }
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }


  DoubleTaskDispatcher::setOutput("first_agent_state",task_feedback_sub_[first_agent_name_]->isANewDataAvailable());
  DoubleTaskDispatcher::setOutput("second_agent_state",task_feedback_sub_[second_agent_name_]->isANewDataAvailable());

  return BT::NodeStatus::SUCCESS;
}

DoubleTaskDispatcher::~DoubleTaskDispatcher()
{
}
