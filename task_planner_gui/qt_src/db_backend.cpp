#include "db_backend.h"


DbUserBackEnd::DbUserBackEnd(ros::NodeHandle nh, QString id) : BackEnd(nh, id)
{
  m_id = id;

  loadDataFromDB();
}


void DbUserBackEnd::loadDataFromDB(){

  QString driver = "QMYSQL";
  QString dbName = "data_task_planner_db";

  if (QSqlDatabase::isDriverAvailable(driver))
    ROS_WARN("Driver QMYSQL available!");
  else
    ROS_ERROR("Driver QMYSQL unavailable");
  QSqlDatabase db = QSqlDatabase::addDatabase(driver);
  db.setDatabaseName(dbName);
  db.setHostName("localhost");
  db.setUserName("root");
  db.setPassword("password");


  if (db.open())
    ROS_WARN("Succesfully connected to database: '%s'", dbName.toStdString().c_str());
  else
    ROS_ERROR("Unable to perform a connection with a database. Please check HostName, UserName, Password and dbName");



  QSqlQuery commandSQL("select * from task");

  if (!commandSQL.exec("select * from task"))
    ROS_ERROR("unable to complete a select from QML");
  else
    while(commandSQL.next())
    {
      QString adw = commandSQL.value("name").toString();
    }

  QString name_robot;
  bool is_name_registered = false;
  if (!commandSQL.exec("select * from robots"))
    ROS_ERROR("unable to complete a select from robots QML");
  else
  {
    while(commandSQL.next())
    {
      name_robot = commandSQL.value("name").toString();
      if (name_robot.toStdString() == m_id.toStdString())
      {
        is_name_registered = true;
        break;
      }
    }
    if (!is_name_registered)
    {
      QString query_Qstr = QString("insert into robots (name) values('%1');").arg(m_id);
      if (!commandSQL.exec(query_Qstr))
        ROS_ERROR("unable to complete an insert to table robots from QML");
    }

  }

  QString query_Qstr = QString("select id from robots where name = '%1'").arg(m_id);
  if (!commandSQL.exec(query_Qstr))
    ROS_ERROR("unable to complete a select from QML");
  else
    while(commandSQL.next())
    {
      QString adw = commandSQL.value("id").toString();
      m_robot_id = adw.toInt();
    }

  commandSQL = QSqlQuery("select max(id) from sessions");
  commandSQL.next();
  QString adw = commandSQL.value("max(id)").toString();

  m_session_id = adw.toInt();
  m_session_id++;  //Update session number

  QString curr_date = QDateTime::currentDateTime().toString("ddd MMM d yyyy");
  QString curr_clock = QDateTime::currentDateTime().toString("hh:mm:ss");
  QString query_str = QString("insert into sessions (id,date,clock) values(%1, '%2', '%3');").arg(m_session_id).arg(curr_date).arg(curr_clock);
  if (!commandSQL.exec(query_str))
  {
    ROS_ERROR("unable to complete an insert to table sessions from QML");
  }

}


void DbUserBackEnd::updateSQLSlotForced()
{

  m_avg_distance = m_cur_distance/m_count_dist;
  m_avg_speed_ovr = m_cur_speed_ovr/m_count_speed_ovr;

  if (std::isnan(m_avg_distance))
    m_avg_distance = 0.0;
  if (std::isnan(m_avg_speed_ovr))
    m_avg_speed_ovr = 0.0;

  std::cout << "m_avg_distance " << m_avg_distance << std::endl;
  std::cout << "m_avg_speed_ovr " << m_avg_speed_ovr << std::endl;

  double time_end = ros::Time::now().toSec() - BackEnd::getTimeReceivedTask();
  double time_max = 100.0;
  double dist_max = 5.0;
  if (time_end > time_max)
    time_end = time_max;
  if (m_avg_distance > dist_max)
    m_avg_distance = dist_max;

  //Add new task to SQL DB
  task_planner_interface_msgs::MotionTaskExecutionRequest task = BackEnd::getFirstTask();

  QSqlQuery commandSQL;
  std::string task_id = "'"+BackEnd::getFirstTaskId()+"'";
  std::string name = "'"+task.task_name+"'";
  std::string start = "'"+task.cfg_start+"'";
  std::string goal = "'"+task.cfg_goal+"'";
  std::string status =  "'"+m_status.toStdString()+"'";

  std::string time;
  if (m_status == "SUCCESS")
   time = "'"+std::to_string(time_end)+"'";
  else
   time = "'0.0'";

  std::string distance_hr = "'"+std::to_string(m_avg_distance)+"'";
  std::string override_robot = "'"+std::to_string(m_avg_speed_ovr)+"'";

  std::string session_id = std::to_string(m_session_id);
  std::string robot_id = std::to_string(m_robot_id);

  std::cout << task_id << std::endl;
  std::cout << name << std::endl;
  std::cout << start << std::endl;
  std::cout << goal << std::endl;
  std::cout << time << std::endl;
  std::cout << status << std::endl;
  std::cout << distance_hr << std::endl;
  std::cout << override_robot << std::endl;
  std::cout << session_id << std::endl;
  std::cout << robot_id << std::endl;

  ROS_ERROR("qua");
  std::string  qr= "insert into task (task_id,name,start,goal,time,status,distance_hr,override_robot,session_id,robot_id) values("
      +task_id+","+name+","+start+","+goal+","+time+","+status+","+distance_hr+","+override_robot+","+session_id+","+robot_id+");";

  QString query_str = qr.c_str();
  if (!commandSQL.exec(query_str))
  {
   ROS_ERROR("unable to complete queery 'insert into task ...'");
  }
  ROS_ERROR("quas");

}


void DbUserBackEnd::updateSQLSlot(const QString &msg, const QString &result)
{

  m_status = result;

}













