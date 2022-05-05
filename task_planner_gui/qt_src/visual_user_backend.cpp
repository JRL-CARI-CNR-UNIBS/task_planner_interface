#include "visual_user_backend.h"


VisualUserBackEnd::VisualUserBackEnd(ros::NodeHandle nh)
{
  m_model=new QStandardItemModel(this);
  m_model->insertColumn(0);


}


void VisualUserBackEnd::selectDateSlot(const QString &msg){

  int i = 0;
  m_model->removeRows(0,m_model->rowCount());
  const int total_rows= 0;

  QSqlQuery commandSQL;
  QString query_Qstr = QString("SELECT id, clock FROM sessions WHERE date = '%1' ORDER BY id").arg(msg);
  if (!commandSQL.exec(query_Qstr))
    ROS_ERROR("unable to complete a select from QML");
  else
    while(commandSQL.next())
    {
      m_session.setID(commandSQL.value("id").toInt());
      m_session.setClock(commandSQL.value("clock").toString());

      m_model->insertRow(total_rows+i);
      m_model->setData(m_model->index(total_rows+i,0),QVariant::fromValue(m_session),Qt::EditRole);
      emit m_model->dataChanged(m_model->index(total_rows+i,0),m_model->index(0,0));
      i++;
    }

}

void VisualUserBackEnd::selectSessionSlot(const QString &msg)
{

  //Get Time from DB
  QSqlQuery commandSQL;
  QString query_Qstr = QString("SELECT task_id, name, start, goal, time,override_robot,distance_hr FROM task WHERE session_id = '%1' ORDER BY id").arg(msg);
  if (!commandSQL.exec(query_Qstr))
    ROS_ERROR("unable to complete a select from QML");
  else
  {
    m_time.resize(0);
    m_task_id.resize(0);
    m_name.resize(0);
    m_start.resize(0);
    m_goal.resize(0);
    m_distance_hr.resize(0);
    m_speed_ovr.resize(0);
    while(commandSQL.next())
    {
      m_task_id.push_back(commandSQL.value("task_id").toString());
      m_name.push_back(commandSQL.value("name").toString());
      m_start.push_back(commandSQL.value("start").toString());
      m_goal.push_back(commandSQL.value("goal").toString());
      m_time.push_back(commandSQL.value("time").toDouble());

      m_distance_hr.push_back(commandSQL.value("distance_hr").toDouble());
      m_speed_ovr.push_back(commandSQL.value("override_robot").toDouble());
    }
  }

  if (m_time.size() != 0) {
    m_max_time = 1.1*(*std::max_element(m_time.begin(),m_time.end()));
    m_min_time = 0.9*(*std::min_element(m_time.begin(),m_time.end()));
  }
  else {
    m_max_time = 0.0;
    m_min_time = 0.0;
  }

  if (m_distance_hr.size() != 0) {
    m_max_distance_hr = 1.1*(*std::max_element(m_distance_hr.begin(),m_distance_hr.end()));
    m_min_distance_hr = 0.9*(*std::min_element(m_distance_hr.begin(),m_distance_hr.end()));
  }
  else {
    m_max_distance_hr = 0.0;
    m_min_distance_hr = 0.0;
  }

  if (m_speed_ovr.size() != 0) {
    m_max_speed_ovr = 1.1*(*std::max_element(m_speed_ovr.begin(),m_speed_ovr.end()));
    m_min_speed_ovr = 0.9*(*std::min_element(m_speed_ovr.begin(),m_speed_ovr.end()));
  }
  else {
    m_max_speed_ovr = 0.0;
    m_min_speed_ovr = 0.0;
  }


  m_category_task.clear();
  m_task_bar_set.clear();
  m_time_bar_set.clear();

  if (!m_task_id.empty())
    for (int i=0; i < m_task_id.size(); i++)
    {
      m_category_task.push_back(QString("T%1").arg(i));
      m_task_bar_set.append(m_distance_hr.at(i));
      m_time_bar_set.append(m_time.at(i));
    }

  emit updatePlotDataSig();
  ROS_WARN("updatePlotDataSig emitted");
}


void VisualUserBackEnd::removeSessionSlot(const int &msg_session_id, const int &msg_idx_lv)
{

  QSqlQuery commandSQL;
  QString query_Qstr = QString("DELETE FROM sessions WHERE id = '%1'").arg(msg_session_id);
  if (!commandSQL.exec(query_Qstr))
    ROS_ERROR("unable to complete QML query: DELETE FROM sessions ...");
  else
  {
    m_model->removeRow(msg_idx_lv);
  }


}



