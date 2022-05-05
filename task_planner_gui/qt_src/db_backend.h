#ifndef db_backend_2212
#define db_backend_2212

//An extension of the UserBackEnd class which includes data
//managements with MySQL database and additional features.
//The class will run only if a new DB is started previously.


#include "user_backend.h"
#include "robot_backend.h"
#include <ros/ros.h>
#include <QtSql>
#include <QObject>


class DbUserBackEnd : public BackEnd
{
  Q_OBJECT

public:

  DbUserBackEnd(ros::NodeHandle nh, QString id);

  void loadDataFromDB();

public Q_SLOTS:
  void updateSQLSlot(const QString &msg, const QString &result);
  void updateSQLSlotForced();

protected:


signals:


private:
  QString m_id;
  int m_robot_id;
  int m_session_id;
  QString m_status;




};

#endif // DbUserBackEnd_H
