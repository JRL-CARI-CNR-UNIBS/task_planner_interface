#ifndef backenduser_01581604
#define backenduser_01581604

#include <QtSql>

#include <QObject>
#include <QString>
#include <QDate>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <subscription_notifier/subscription_notifier.h>
#include <QStandardItemModel>
#include <std_srvs/SetBool.h>
#include <string>
#include <task_planner_interface_msgs/MotionTaskExecutionFeedback.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequest.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequestArray.h>


class TaskUser
{
    Q_GADGET //Dovrebbe essere una versione light di Q_OBJECT
    Q_PROPERTY(QString id READ id)
    Q_PROPERTY(QString name READ name)
    Q_PROPERTY(QString startName READ startName)
    Q_PROPERTY(QString goalName READ goalName)
    Q_PROPERTY(QString color READ color)// WRITE setColor)
    Q_PROPERTY(int height READ height)
    Q_PROPERTY(QString colorLabel READ colorLabel)
    Q_PROPERTY(double proximity READ proximity)
    Q_PROPERTY(double expectedTime READ expectedTime)
    Q_PROPERTY(QString proximityColor READ proximityColor)

public:
    const QString& id() const { return m_id; }
    const QString& name() const { return m_name; }
    const QString& startName() const { return m_start_name; }
    const QString& goalName() const { return m_goal_name; }
    const QString& color() const { return m_color; }
    const QString& colorLabel() const { return m_color_label; }
    const int& height() const { return m_height; }
    const double& proximity() const { return m_proximity; }
    const QString& proximityColor() const { return m_proximity_color; }
    const double& expectedTime() const { return m_expected_time; }
    void setId(QString id){m_id = id;}
    void setName(QString name){m_name = name;}
    void setStartName(QString start_name){m_start_name = start_name;}
    void setGoalName(QString goal_name){m_goal_name = goal_name;}
    void setColorLabel(QString color_label){m_color_label = color_label;}
    void setColor(QString color){m_color = color;}
    void setHeight(int height){m_height = height;}
    void setProximity(double proximity){m_proximity = proximity;}
    void setExpectedTime(double expected_time){m_expected_time = expected_time;}
    void setProximityColor(QString proximity_color){m_proximity_color = proximity_color;}


private:
    QString m_id = "Default Id";
    QString m_name = "Default TaskUser";
    QString m_start_name = "A1";
    QString m_goal_name = "A2";
    QString m_color_label = "#fcb7a2";
    QString m_color = "#3285a8";
    int m_height = 200;
    double m_proximity;
    double m_expected_time;
    QString m_proximity_color = "#89989c";

};


//Register TaskUser class for retrieving and obtain data from QVariant (must be places in namespace)
Q_DECLARE_METATYPE(TaskUser)

class BackEndUser : public QObject
{
  Q_OBJECT

  Q_PROPERTY(QAbstractItemModel* model READ model NOTIFY modelChanged)
  Q_PROPERTY(bool newTask READ newTask NOTIFY newTaskChanged)
  Q_PROPERTY(QString agentName READ agentName NOTIFY agentNameChanged)


public:

  explicit BackEndUser(ros::NodeHandle nh, QString id, QObject *parent = nullptr);

  QAbstractItemModel* model() const {return m_model;}
  void newCommandCallback(const task_planner_interface_msgs::MotionTaskExecutionRequestArrayConstPtr& msg);
  void deployDummyTask(double risk, QString name);
  bool newTask();

  QString agentName(){return m_id;}
  bool newRows();

//  void caricaDatiDalDb();


public Q_SLOTS:
  void acceptedUserTaskSlot(const QString &msg);
  void removeItemSlot(const QString &msg, const QString &result);


signals:
  void newDataCallbackSig();
  void modelChanged();
  void agentNameChanged();
  void mediumlProximitySig();
  void highProximitySig();
  void stopHighProximitySig();
  void remItemSig();
  void forceRemove();
  void actionButtonsActive();
  bool newTaskChanged();

protected:
  task_planner_interface_msgs::MotionTaskExecutionRequest getFirstTask(){return m_task_list_msg.at(0);}
  std::string getFirstTaskId(){return m_task_id;}
  int getCmdId(){return m_cmd_id;}
  double getTimeReceivedTask(){return time_begin;}

private:


  ros::NodeHandle m_nh;
  ros::Subscriber m_sub1;
  ros::Subscriber m_sub2;
  ros::Publisher m_pub;
  task_planner_interface_msgs::MotionTaskExecutionFeedback m_msg;

  ros::ServiceServer m_srv_ser;

  QString m_id;
  std::string m_task_id;
  int m_cmd_id;

  QAbstractItemModel* m_model;
  TaskUser m_task = TaskUser();
  TaskUser m_task0;
  std::vector<task_planner_interface_msgs::MotionTaskExecutionRequest> m_task_list_msg;
  std::vector<double> m_risk_levels;

  bool m_allow_insert_rows = true;

  double time_begin;

  //For SQL
  int m_session_id;
  int m_user_id;






};

#endif // BackEndUser_H
