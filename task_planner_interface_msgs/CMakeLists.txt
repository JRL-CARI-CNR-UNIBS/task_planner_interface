cmake_minimum_required(VERSION 3.0.2)
project(task_planner_interface_msgs)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  message_runtime
  actionlib_msgs
)

add_message_files(
  FILES
  MotionTaskExecutionFeedback.msg
  MotionTaskExecutionRequest.msg
  MotionTaskExecutionRequestArray.msg
  TaskAgents.msg
  TaskStatistics.msg
)

add_service_files(
  FILES
  TaskResult.srv
  TaskType.srv
  BasicSkill.srv
  PickPlaceSkill.srv
  PauseSkill.srv
  TasksInformation.srv
  TasksStatistics.srv
)
add_action_files(
  DIRECTORY action
  FILES
  TaskExecute.action
)
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
