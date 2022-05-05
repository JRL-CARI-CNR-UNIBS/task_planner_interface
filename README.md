TASK-PLANNER-INTERFACE
====================

This repo groups packages for the communication between timeline-based task planners and an action planner for robot manipulation.
The software was developed within the European Project [Sharework](https://sharework-project.eu/). 

Packages:

 - task_planner_interface: interface between Platinum and the manipulation package
 - task_planner_interface_msgs: definition of custom messages for the communication of packages
 - task_planner_dispatcher: talker nodes to dispatch task plans (dummy nodes to simulate a task planner)
 - task_planner_gui: gui to show task requests and give feedback on the task execution (for real experiments)
 - sharework_imu_hmi: simple node to give feedback on the task execution through the armband myo
 - sharework_task_planner: launch and config files to use the packages with Sharework use case
 
## Requirements

 - package ``task_planner_interface`` assumes package **manipulation** is installed and an instance of **mongod is running** (see [here](https://bitbucket.org/CNR-ITIA/task-planner-interface/src/d7cf9a4e0726/task_planner_interface/?at=master) for details)
 - task_planner_gui requires **Qml and MySQL**. See [here](https://bitbucket.org/CNR-ITIA/task-planner-interface/src/master/task_planner_gui/) for details
 - sharework_imu_hmi is intended to be used along with [ros_myo](https://github.com/JRL-CARI-CNR-UNIBS/ros_myo?organization=JRL-CARI-CNR-UNIBS&organization=JRL-CARI-CNR-UNIBS)
 
 **Note:** task-planner-interface is thought to interface ROS actions from meta-package ``manipulation`` with a task planner that only communicates through the messages defined in task_planner_interface_msgs. If a task planner is not running, it is possible to read and dispatch recipes through ``task_planner_dispatcher`` package.
 In Sharework, the task planner works outside ROS and communicates with ROS via the [rosbridge_suite](http://wiki.ros.org/rosbridge_suite) package.