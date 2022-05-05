#ifndef backend_01581604
#define backend_01581604


#include <QObject>
#include <QString>
#include <QDate>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <subscription_notifier/subscription_notifier.h>
#include <QStandardItemModel>
#include <std_srvs/SetBool.h>
#include <task_planner_interface_msgs//MotionTaskExecutionFeedback.h>
#include <task_planner_interface_msgs//MotionTaskExecutionRequest.h>
#include <task_planner_interface_msgs//MotionTaskExecutionRequestArray.h>


class Task
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
    Q_PROPERTY(bool visibleItem READ visibleItem)

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
    const bool& visibleItem() const { return m_visible_item; }
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
    void setVisibleItem(bool visible_item){m_visible_item = visible_item;}


private:
    QString m_id = "Default Id";
    QString m_name = "Default Task";
    QString m_start_name = "A1";
    QString m_goal_name = "A2";
    QString m_color_label = "#fcb7a2";
    QString m_color = "#3285a8";
    int m_height = 200;
    double m_proximity;
    double m_expected_time;
    QString m_proximity_color = "#89989c";
    bool m_visible_item;



};


//Register Task class for retrieving and obtain data from QVariant (must be places in namespace)
Q_DECLARE_METATYPE(Task)

class BackEnd : public QObject
{
  Q_OBJECT

  Q_PROPERTY(QAbstractItemModel* model READ model NOTIFY modelChanged)
//  Q_PROPERTY(bool newTask READ newTask)
  Q_PROPERTY(QString agentName READ agentName NOTIFY agentNameChanged)


public:

  explicit BackEnd(ros::NodeHandle nh, QString id, QObject *parent = nullptr);

  QAbstractItemModel* model() const {return m_model;}

  void endCommandCallback(const task_planner_interface_msgs::MotionTaskExecutionFeedback& msg);
  void deployDummyTask(double risk, QString name);

  QString agentName(){return m_id;}





public Q_SLOTS:
  void newTask();
  void removeItemSlot(const QString &msg, const QString &result);
  void endRemoveItemAnimationSlot(const QString &msg);
  void newCommandCallback(const task_planner_interface_msgs::MotionTaskExecutionRequestArrayConstPtr& msg);
  void overrideSlot(const QString& msg);

  void distanceCallback(const std_msgs::Float64ConstPtr& msg);
  void speedOvrCallback(const std_msgs::Int64ConstPtr& msg);

protected:
  task_planner_interface_msgs::MotionTaskExecutionRequest getFirstTask(){return m_task_list_msg.at(0);}
  std::string getFirstTaskId(){return m_task_id;}
  double getTimeReceivedTask(){return m_time_begin;}
  double getAvgDistance(){return m_avg_distance;}
  double getAvgSpeedOvr(){return m_avg_speed_ovr;}

  double m_avg_distance = 0.0;
  double m_avg_speed_ovr = 0.0;

  double m_cur_distance;
  double m_cur_speed_ovr;

  int m_count_dist = 0;
  int m_count_speed_ovr = 0;

  bool m_active_count = false;



signals:
  void newDataCallbackSig();
  void modelChanged();  
  void agentNameChanged();
  void mediumlProximitySig();
  void highProximitySig();
  void stopHighProximitySig();
  void remItemSig();
  void forceRemove();
  void confirmRemoveSig();
  void startRemoveSig();
  void forceRemoveAnim();

private:

  ros::NodeHandle m_nh;
  ros::Subscriber m_sub1;
  ros::Subscriber m_sub2;
  ros::Subscriber m_sub3;
  ros::Subscriber m_sub4;
  ros::Publisher m_ovr_pub;
  std_msgs::Int64 m_msg_ovr;
  ros::ServiceServer m_srv_ser;

  QString m_id;
  std::string m_task_id;

  QAbstractItemModel* m_model;
  Task m_task = Task();
  Task m_task0;
  std::vector<task_planner_interface_msgs::MotionTaskExecutionRequest> m_task_list_msg;
  std::vector<double> m_risk_levels;

  bool m_allow_insert_rows = true;

  double m_time_begin;


};

#endif // BACKEND_H
