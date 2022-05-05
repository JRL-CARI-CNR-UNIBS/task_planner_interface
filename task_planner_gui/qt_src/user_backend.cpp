#include "user_backend.h"


BackEndUser::BackEndUser(ros::NodeHandle nh, QString id, QObject *parent) :
    QObject(parent)
{
  m_nh = nh;
  m_id = id;

  m_model=new QStandardItemModel(this);
  m_risk_levels = {0.33,0.66,1.0};
  m_model->insertColumn(0);

  //If will be removed
  if (m_id.toStdString() == "user1")
  {
    std::string topic_motion_feedback = "/sharework/test/stiima/gui/human/feedback";
    if (!m_nh.getParam("topic_motion_feedback",topic_motion_feedback))
    {
      ROS_ERROR("topic_motion_feedback not defined, set a default topic name: %s", topic_motion_feedback.c_str());
    }
    std::string topic_motion_request = "/sharework/test/stiima/human";
    m_pub = m_nh.advertise<task_planner_interface_msgs::MotionTaskExecutionFeedback>(topic_motion_feedback,1);
    m_sub1 = m_nh.subscribe(topic_motion_request,1,&BackEndUser::newCommandCallback,this);
  }
  else
  {
    deployDummyTask(0.1,"Pick");
    deployDummyTask(0.1,"Place");
    deployDummyTask(0.1,"Inspection");
    deployDummyTask(0.1,"Assemble");
  }



  ROS_INFO("Class correctly constructed");

}


void BackEndUser::deployDummyTask(double risk, QString name)
{

  const int newRow= m_model->rowCount();
  m_model->insertRow(newRow);

  m_task.setHeight(50);
  m_task.setProximity(risk);
  m_task.setName(name);
  m_task.setGoalName("Blue");

  m_model->setData(m_model->index(newRow,0),QVariant::fromValue(m_task),Qt::EditRole);
  emit m_model->dataChanged(m_model->index(newRow,0),m_model->index(0,0));

}


bool BackEndUser::newRows()
{

}
bool BackEndUser::newTask()
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

    m_task.setId(QString(m_task_id.c_str()));
    m_task.setName(QString(cmd_name.c_str()));
    m_task.setGoalName(QString(goal_name.c_str()));
    m_task.setStartName(QString(start_name.c_str()));
    m_task.setHeight(50);
    m_task.setProximity(0.1); //No risk factor, human case

    m_model->setData(m_model->index(i,0),QVariant::fromValue(m_task),Qt::EditRole);
    emit m_model->dataChanged(m_model->index(i,0),m_model->index(0,0));


  }

  time_begin = ros::Time::now().toSec();

  emit actionButtonsActive();

  return true;

}


void BackEndUser::acceptedUserTaskSlot(const QString &msg){
  //Only User type class: Publish accepted feedback
  m_msg.cmd_id = m_cmd_id;
  m_msg.result = 2;
  m_pub.publish(m_msg);
}


void BackEndUser::removeItemSlot(const QString &msg, const QString &result){

  if (m_model->rowCount() != 0)
  {
    //Publish feedback
    m_msg.cmd_id = m_cmd_id;
    if (result == "SUCCESS")
      m_msg.result = 1;
    else
      m_msg.result = 0;
    m_pub.publish(m_msg);

    //Remove from model
    m_model->removeRow(0);

    ROS_WARN("Item %d correctly removed", 0);

  }
  else
  {
    ROS_ERROR("Unable to remove the itme, no rows are available");
  }

}



void BackEndUser::newCommandCallback (const task_planner_interface_msgs::MotionTaskExecutionRequestArrayConstPtr& msg)
{

  ROS_WARN("New data via callback at c++");

  m_task_list_msg.resize(msg->tasks.size());
  m_cmd_id = msg->cmd_id;

  for (unsigned int i=0; i<msg->tasks.size(); i++)
    m_task_list_msg.at(i) = msg->tasks.at(i);

  emit newDataCallbackSig();

}






