#ifndef visual_user_backend_0317
#define visual_user_backend_0317

//An extension of the UserBackEnd class which includes data
//managements with MySQL database and additional features.
//The class will run only if a new DB is started previously.


#include "user_backend.h"
#include <ros/ros.h>
#include <QtSql>
#include <QObject>
#include <QLineSeries>
//#include <QBarSet>
#include <QtCharts>

class Session {
  Q_GADGET //Dovrebbe essere una versione light di Q_OBJECT
  Q_PROPERTY(int id READ id)
  Q_PROPERTY(QString name READ name)
  Q_PROPERTY(QString clock READ clock)

  public:
    const int id() const { return m_id; }
    const QString name() const { return m_name; }
    const QString clock() const { return m_clock; }
    int setID(int id){m_id = id;}
    void setName(QString name){m_name = name;}
    void setClock(QString clock){m_clock = clock;}

  private:
    int m_id;
    QString m_name;
    QString m_clock;
};


//Register TaskUser class for retrieving and obtain data from QVariant (must be places in namespace)
Q_DECLARE_METATYPE(Session)

class VisualUserBackEnd : public QObject
{
  Q_OBJECT
  Q_PROPERTY(QAbstractItemModel* model READ model NOTIFY modelChanged)
  Q_PROPERTY(std::vector<double> plotData READ plotData NOTIFY plotDataChanged)
//  Q_PROPERTY(int plotCount READ plotCount NOTIFY plotCountChanged)




public:

  VisualUserBackEnd(ros::NodeHandle nh);
  QAbstractItemModel* model() const {return m_model;}
  std::vector<double> plotData() {return m_plotData;}

  Q_INVOKABLE int plotCount(){return ( (m_task_id.empty()) ? (0) : m_task_id.size() );}
  Q_INVOKABLE double getTime(int i){return m_time.at(i);}
  Q_INVOKABLE double getMaxTime(){return m_max_time;}
  Q_INVOKABLE double getMinTime(){return m_min_time;}
  Q_INVOKABLE double getDistanceHR(int i){return m_distance_hr.at(i);}
  Q_INVOKABLE double getMaxDistanceHR(){return m_max_distance_hr;}
  Q_INVOKABLE double getMinDistanceHR(){return m_min_distance_hr;}
  Q_INVOKABLE double getSpeedOvr(int i){return m_speed_ovr.at(i);}
  Q_INVOKABLE double getMaxSpeedOvr(){return m_max_speed_ovr;}
  Q_INVOKABLE double getMinSpeedOvr(){return m_min_speed_ovr;}

  Q_INVOKABLE QStringList getCategoryTasks(){return m_category_task;}
  Q_INVOKABLE QVariantList getTasksBarSet(){return m_task_bar_set;}
  Q_INVOKABLE QVariantList getTimeTasksBarSet(){return m_time_bar_set;}
  Q_INVOKABLE QString getTaskID(int i){return m_task_id.at(i);}
  Q_INVOKABLE QString getTaskName(int i){return m_name.at(i);}
  Q_INVOKABLE QString getStart(int i){return m_start.at(i);}
  Q_INVOKABLE QString getGoal(int i){return m_goal.at(i);}



public Q_SLOTS:
  void selectDateSlot(const QString &msg);
  void selectSessionSlot(const QString &msg);
  void removeSessionSlot(const int &msg_session_id, const int &msg_idx_lv);

protected:


signals:
  void modelChanged();
  void plotDataChanged();
  void plotCountChanged();
  void updatePlotDataSig();


private:
  std::vector<double> m_plotData;
  QString m_id;
  int m_user_id;
  int m_session_id;
  QAbstractItemModel* m_model;
  std::vector<double> m_time;
  double m_max_time;
  double m_min_time;

  std::vector<double> m_distance_hr;
  double m_max_distance_hr;
  double m_min_distance_hr;
  std::vector<double> m_speed_ovr;
  double m_max_speed_ovr;
  double m_min_speed_ovr;
  QStringList m_category_task;

  QVariantList m_task_bar_set;
  QVariantList m_time_bar_set;

  std::vector<QString> m_task_id;
  std::vector<QString> m_name;
  std::vector<QString> m_start;
  std::vector<QString> m_goal;

  Session m_session;






};

#endif // VisualUserBackEnd_H
