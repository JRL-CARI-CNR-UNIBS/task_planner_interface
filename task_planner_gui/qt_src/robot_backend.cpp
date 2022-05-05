#include "robot_backend.h"


BackEnd::BackEnd(ros::NodeHandle nh, QString id, QObject *parent) :
    QObject(parent)
{
  m_nh = nh;
  m_id = id;
  m_model=new QStandardItemModel(this);
  m_risk_levels = {0.33,0.66,1.0};
  m_model->insertColumn(0);


  if (m_id.toStdString() == "ur5")
  {
    std::string topic_safety_override = "/sharework/test/stiima/motion/safe_ovr";
    if (!m_nh.getParam("safety_override",topic_safety_override))
    {
      ROS_ERROR("safety_override not defined, set a default topic name: %s", topic_safety_override.c_str());
    }
    std::string topic_motion_feedback = "/sharework/test/stiima/motion/feedback";
    std::string topic_motion_request = "/sharework/test/stiima/motion/";
    std::string topic_distance = "/distance";
    std::string topic_speed_ovr = "/speed_ovr_log";
    m_sub1 = m_nh.subscribe(topic_motion_request,1,&BackEnd::newCommandCallback,this);
    m_sub2 = m_nh.subscribe(topic_motion_feedback,1,&BackEnd::endCommandCallback,this);
    m_sub3 = m_nh.subscribe(topic_distance,1,&BackEnd::distanceCallback,this);
    m_sub4 = m_nh.subscribe(topic_speed_ovr,1,&BackEnd::speedOvrCallback,this);
    m_ovr_pub=m_nh.advertise<std_msgs::Int64>(topic_safety_override,1);
  }
  else
  {
    deployDummyTask(0.2,"Pick");
    deployDummyTask(1.0,"Place");
    deployDummyTask(0.2,"Inspection");
    deployDummyTask(0.2,"Assemble");
  }



  ROS_INFO("Class correctly constructed");

}

void BackEnd::deployDummyTask(double risk, QString name)
{
  const int newRow= m_model->rowCount();
  m_model->insertRow(newRow);

  m_task.setProximity(risk);
  m_task.setName(name);
  m_task.setGoalName("Init Box");

  m_model->setData(m_model->index(newRow,0),QVariant::fromValue(m_task),Qt::EditRole);
  emit m_model->dataChanged(m_model->index(newRow,0),m_model->index(0,0));
}


void BackEnd::newTask()
{
  if (m_model->rowCount() != m_task_list_msg.size())
    for (unsigned int i=m_model->rowCount(); i < m_task_list_msg.size(); i++)
      m_model->insertRow(m_model->rowCount());


  for (unsigned int i=0; i<m_task_list_msg.size(); i++)
  {
    task_planner_interface_msgs::MotionTaskExecutionRequest task = m_task_list_msg.at(i);
    std::string cmd_name = task.task_name;
    std::string goal_name = task.cfg_goal;
    std::string start_name = task.cfg_start;
    double expected_time = task.expected_time;
    double proximity_leve = task.risk_level;
    double scaled_height_factor = 15.0;

    m_task.setId(QString(m_task_id.c_str()));
    m_task.setName(QString(cmd_name.c_str()));
    m_task.setGoalName(QString(goal_name.c_str()));
    m_task.setStartName(QString(start_name.c_str()));
    m_task.setExpectedTime(expected_time); //Time in seconds
    m_task.setHeight(expected_time*scaled_height_factor);
    m_task.setProximity(proximity_leve);
    m_task.setVisibleItem(true);

    m_model->setData(m_model->index(i,0),QVariant::fromValue(m_task),Qt::EditRole);
    emit m_model->dataChanged(m_model->index(i,0),m_model->index(0,0));
  }

  m_time_begin = ros::Time::now().toSec();

  //Start removing animation on task 0
  emit startRemoveSig();

}


void BackEnd::newCommandCallback (const task_planner_interface_msgs::MotionTaskExecutionRequestArrayConstPtr& msg)
{
  ROS_WARN("New data from callback");

  m_task_id = msg->cmd_id;
  m_task_list_msg.resize(msg->tasks.size());
  for (unsigned int i=0; i<msg->tasks.size(); i++)
    m_task_list_msg.at(i) = msg->tasks.at(i);

  //First, reset current task animations that are still active with a forceRemove signal
  emit forceRemove();

  //Emit visualize new tasks
  emit newDataCallbackSig(); //First creation of items

  m_active_count = true;
  m_count_dist = 0;
  m_count_speed_ovr = 0;
  m_cur_distance = 0.0;
  m_cur_speed_ovr = 0.0;


}


void BackEnd::endCommandCallback(const task_planner_interface_msgs::MotionTaskExecutionFeedback& msg)
{
  ROS_WARN("End request, starting a force remove");
  emit forceRemove();
  emit forceRemoveAnim();
  m_active_count = false;
//  m_avg_distance = m_cur_distance/m_count_dist;
//  m_avg_speed_ovr = m_cur_speed_ovr/m_count_speed_ovr;

//  std::cout << "m_avg_distance " << m_avg_distance << std::endl;
//  std::cout << "m_avg_speed_ovr " << m_avg_speed_ovr << std::endl;
}


//SLOTS
void BackEnd::removeItemSlot(const QString &msg, const QString &result)
{

  QVariant model_variant;

  if (m_model->rowCount() != 0)
  {
    //Only robot (motion) type class: Send allert signal if next task is risk
    model_variant = m_model->data(m_model->index(0,0), Qt::EditRole);
    m_task0 = model_variant.value<Task>();
    if (m_task0.proximity() > m_risk_levels.at(1))
      emit highProximitySig();

    //Remove from model
    ROS_WARN("Start removing Task");
    m_model->removeRow(0);
    emit remItemSig();
    ROS_WARN("Task data correctly removed");

  }
  else
    ROS_ERROR("Unable to remove the item, no rows are available");
}

void BackEnd::endRemoveItemAnimationSlot(const QString &msg)
{
  if (m_task0.proximity() > m_risk_levels.at(1))
    emit stopHighProximitySig();
}

void BackEnd::overrideSlot(const QString& msg){

  if (msg.toStdString() == "STOP")
    m_msg_ovr.data = 0.0;
  else
    m_msg_ovr.data = 1.0;

  m_ovr_pub.publish(m_msg_ovr);

}


void BackEnd::distanceCallback(const std_msgs::Float64ConstPtr& msg)
{
  m_cur_distance = m_cur_distance + msg->data;

  if (m_active_count)
    m_count_dist++;
//  ROS_ERROR("qui");
}

void BackEnd::speedOvrCallback(const std_msgs::Int64ConstPtr& msg)
{
  m_cur_speed_ovr = m_cur_speed_ovr + msg->data;

  if (m_active_count)
    m_count_speed_ovr++;

//  ROS_ERROR("qua");
}

