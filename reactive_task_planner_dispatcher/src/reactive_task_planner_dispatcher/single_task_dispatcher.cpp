
#include "reactive_task_planner_dispatcher/single_task_dispatcher.h"


SingleTaskDispatcher::SingleTaskDispatcher(const std::string &name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
{
  /* Input from behavior scheme */
  if(!SingleTaskDispatcher::getInput<std::string>("task_name",task_name_))
  {
    ROS_ERROR("Missing required input [task_name_]");
    throw BT::RuntimeError("Missing required input [task_name_]");
  }
  if(!SingleTaskDispatcher::getInput<std::string>("agent_name",agent_name_))
  {
    ROS_ERROR("Missing required input [task_name_]");
    throw BT::RuntimeError("Missing required input [task_name_]");
  }

//  ROS_INFO_STREAM("Task Name: " << task_name_);
//  ROS_INFO_STREAM("Agent Name: " << agent_name_);

  /* Nedded parameters */
  std::string task_feedback_topic_name;
  if (!nh_.getParam("task_feedback_"+agent_name_,task_feedback_topic_name))
  {
    ROS_ERROR_STREAM("task_feedback_" << agent_name_ << " not defined");
    throw BT::RuntimeError("Missing required parameter task_feedback_");
  }
  std::string task_request_topic_name;
  if (!nh_.getParam("task_request_"+agent_name_,task_request_topic_name))
  {
    ROS_ERROR_STREAM("task_request_" << agent_name_ <<" not defined");
    throw BT::RuntimeError("Missing required parameter task_request_");
  }

  /* Subscriber - Publisher */
  task_feedback_sub_.reset(new ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback> (nh_, task_feedback_topic_name,10));
  task_request_pub_ = nh_.advertise<task_planner_interface_msgs::MotionTaskExecutionRequestArray>(task_request_topic_name, 1000);

}

void SingleTaskDispatcher::publishTask(const ros::Publisher &pub,
                                       const std::string &task_name)
{
    task_planner_interface_msgs::MotionTaskExecutionRequestArray task_to_publish;
    task_planner_interface_msgs::MotionTaskExecutionRequest task;
    task.task_id = task_name;
    //task_to_publish.cmd_id = 1;
    task_to_publish.tasks={task};
    pub.publish(task_to_publish);
    ros::spinOnce();
}



BT::NodeStatus SingleTaskDispatcher::tick()
{
  ROS_INFO_STREAM("---------------------------");
  ROS_INFO_STREAM("SingleTaskDispatcher ticked");

  std::string piece_input;
  if(SingleTaskDispatcher::getInput("piece_input",piece_input))
  {
    task_name_+= "-" + piece_input + "-" + agent_name_;
    SingleTaskDispatcher::setOutput("piece_output",piece_input);
    ROS_INFO_STREAM(piece_input);

  }

  ROS_INFO_STREAM("Task Name: " << task_name_);
  ROS_INFO_STREAM("Agent Name: " << agent_name_);

  /* Reset get data */
  if(task_feedback_sub_->isANewDataAvailable())
  {
    task_feedback_sub_->getData();
  }

  /* Publish task request */
  publishTask(task_request_pub_,task_name_);

  /* Wait task response */
  ROS_INFO_STREAM("Waiting task feedback...");
  while(!task_feedback_sub_->isANewDataAvailable() && ros::ok())
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  task_feedback_sub_->getData();

//  SingleTaskDispatcher::setOutput("exchange_info_out","pezzo 1");

  /* If there is exchange info in input put it in output */
  /*
  std::string exchange_info;
  BT::Optional<std::string> msg = SingleTaskDispatcher::getInput<std::string>("exchange_info");
  if(msg)
  {
    SingleTaskDispatcher::setOutput("exchange_info_out","agente due ciaooo");
  }
  */
  ROS_INFO_STREAM("Task " << task_name_ << " executed.");



  return BT::NodeStatus::SUCCESS;
}

SingleTaskDispatcher::~SingleTaskDispatcher()
{
}
