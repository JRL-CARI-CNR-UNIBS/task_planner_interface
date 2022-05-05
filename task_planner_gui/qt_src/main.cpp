#ifndef main_gui_interface_node__201812051146
#define main_gui_interface_node__201812051146

#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQuickStyle>
#include <qqmlengine.h>
#include <qqmlcontext.h>
#include <qqml.h>
#include <ros/ros.h>
//#include <backend.h>  //TODO Why not <header library name>
#include "user_backend.h"
#include "robot_backend.h"
#include "db_backend.h"
#include "visual_user_backend.h"


void newCommandCallback(const task_planner_interface_msgs::MotionTaskExecutionFeedbackConstPtr& msg){}


int main(int argc, char** argv)
{
  ros::init(argc,argv,"task_interface_gui");
  ros::NodeHandle nh("task_interface_gui");

  //Set seed for random numbers
  srand((int)time(0));


  QApplication app(argc, argv);


  QQuickStyle::setStyle("Material");
  QQmlApplicationEngine engine;
  QString id1 = "ur5";
  QString id2 = "user1";

  DbUserBackEnd* backend = new DbUserBackEnd(nh,id1);
  BackEndUser* backend2 = new BackEndUser(nh,id2);
  VisualUserBackEnd* vs_us_backend = new VisualUserBackEnd(nh);



  engine.rootContext()->setContextProperty("backend", backend);
  engine.rootContext()->setContextProperty("backend2", backend2);
  engine.rootContext()->setContextProperty("vs_us_backend", vs_us_backend);

  engine.load(QUrl(QStringLiteral("qrc:/qml_main/main.qml")));

  QObject *homePane = engine.rootObjects().first();




  //Connect signals from QML to C++ slots
  QObject::connect(homePane, SIGNAL(overrideSig(QString)), backend, SLOT(overrideSlot(QString)));
  QObject::connect(homePane, SIGNAL(addNewTask()), backend, SLOT(newTask()));
  QObject::connect(homePane, SIGNAL(removeItemSig1(QString,QString)), backend, SLOT(removeItemSlot(QString,QString)));
//  QObject::connect(homePane, SIGNAL(endRemoveItemAnimationSig(QString)), backend, SLOT(endRemoveItemAnimationSlot(QString)));
  QObject::connect(homePane, SIGNAL(acceptedUserTaskSig1(QString)), backend2, SLOT(acceptedUserTaskSlot(QString)));

  QObject::connect(homePane, SIGNAL(removeItemSig2(QString,QString)), backend2, SLOT(removeItemSlot(QString,QString)));
  QObject::connect(homePane, SIGNAL(removeItemSig1(QString,QString)), backend, SLOT(updateSQLSlot(QString,QString)));
  QObject::connect(homePane, SIGNAL(endRemoveItemAnimationSig()), backend, SLOT(updateSQLSlotForced()));
  QObject::connect(homePane, SIGNAL(selectDateSig(QString)), vs_us_backend, SLOT(selectDateSlot(QString)));
  QObject::connect(homePane, SIGNAL(selectSessionSig(QString)), vs_us_backend, SLOT(selectSessionSlot(QString)));
  QObject::connect(homePane, SIGNAL(removeSessionSig(int,int)), vs_us_backend, SLOT(removeSessionSlot(int,int)));


  ros::AsyncSpinner spinner(1); // Use 1 threads
  spinner.start();

  app.exec();
  spinner.stop();

  return 0;






}




#endif

