#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>

void syncroCallback(const std_msgs::HeaderConstPtr& msg){}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "syncronization_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  int n_agents=-1;
  if (!pnh.getParam("number_of_agents",n_agents)){
    ROS_ERROR_STREAM(pnh.getNamespace() << ": number_of_agents not defined");
    return false;
  }
  if (n_agents<1){
    ROS_ERROR_STREAM(pnh.getNamespace() << ": number_of_agents must be positive");
    return false;
  }
  std::string topic_syncronization_req="syncronization_req";
  if (!nh.getParam("topic_syncronization_request",topic_syncronization_req))
    ROS_ERROR("topic_syncronization_request not defined. Default: syncronization_req");
  std::string topic_syncronization_res="syncronization_res";
  if (!nh.getParam("topic_syncronization_response",topic_syncronization_res))
    ROS_ERROR("topic_syncronization_response not defined. Default: syncronization_res");

  ros::Publisher syncro_res_pub=nh.advertise<std_msgs::Bool>(topic_syncronization_res,1);
  ros_helper::SubscriptionNotifier<std_msgs::Header> syncro_req_notif(nh,topic_syncronization_req,2);
  syncro_req_notif.setAdvancedCallback(&syncroCallback);

  ros::Time reset_time=ros::Time::now();
//  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    // ros::Time t0=ros::Time::now();

    std::vector<std::string> sync_agents;
    sync_agents.clear();
    while (sync_agents.size()<n_agents)
    {
      if (syncro_req_notif.waitForANewData(ros::Duration(90)))
      {
        std::string new_agent=syncro_req_notif.getData().frame_id;
        ros::Time req_time=syncro_req_notif.getData().stamp;
        bool is_present=false;
        for (unsigned int idx=0;idx<sync_agents.size();idx++)
        {
          if ( !sync_agents.at(idx).compare(new_agent) )
          {
            is_present=true;
            break;
          }
        }
        if (is_present==false && req_time >= reset_time-ros::Duration(0.01)  )
          sync_agents.push_back(new_agent);
      }
    }
    std_msgs::BoolPtr syncro_res_msg(new std_msgs::Bool());
    syncro_res_msg->data=true;
    syncro_res_pub.publish(syncro_res_msg);
    reset_time=ros::Time::now();
    // ros::spinOnce();
//    loop_rate.sleep();
    ros::Duration(0.001).sleep();
  }

  ROS_INFO("Syncronization node: exit normally.");
  return 0;
}


