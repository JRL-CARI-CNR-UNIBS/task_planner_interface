# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "task_planner_interface_msgs: 10 messages, 5 services")

set(MSG_I_FLAGS "-Itask_planner_interface_msgs:/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg;-Itask_planner_interface_msgs:/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(task_planner_interface_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionFeedback.msg" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionFeedback.msg" ""
)

get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg" ""
)

get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequestArray.msg" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequestArray.msg" "task_planner_interface_msgs/MotionTaskExecutionRequest"
)

get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteAction.msg" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteAction.msg" "task_planner_interface_msgs/TaskExecuteResult:task_planner_interface_msgs/TaskExecuteActionFeedback:task_planner_interface_msgs/TaskExecuteFeedback:std_msgs/Header:task_planner_interface_msgs/TaskExecuteActionGoal:actionlib_msgs/GoalStatus:task_planner_interface_msgs/TaskExecuteActionResult:actionlib_msgs/GoalID:task_planner_interface_msgs/TaskExecuteGoal"
)

get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg" "actionlib_msgs/GoalID:std_msgs/Header:task_planner_interface_msgs/TaskExecuteGoal"
)

get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg" "task_planner_interface_msgs/TaskExecuteResult:actionlib_msgs/GoalID:std_msgs/Header:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg" "actionlib_msgs/GoalID:std_msgs/Header:task_planner_interface_msgs/TaskExecuteFeedback:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg" ""
)

get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg" ""
)

get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg" ""
)

get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskResult.srv" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskResult.srv" ""
)

get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskType.srv" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskType.srv" ""
)

get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/BasicSkill.srv" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/BasicSkill.srv" ""
)

get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PickPlaceSkill.srv" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PickPlaceSkill.srv" ""
)

get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PauseSkill.srv" NAME_WE)
add_custom_target(_task_planner_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_planner_interface_msgs" "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PauseSkill.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequestArray.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteAction.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)

### Generating Services
_generate_srv_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskResult.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/BasicSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PickPlaceSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_cpp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PauseSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
)

### Generating Module File
_generate_module_cpp(task_planner_interface_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(task_planner_interface_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(task_planner_interface_msgs_generate_messages task_planner_interface_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequestArray.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteAction.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskResult.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskType.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/BasicSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PickPlaceSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PauseSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_cpp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(task_planner_interface_msgs_gencpp)
add_dependencies(task_planner_interface_msgs_gencpp task_planner_interface_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS task_planner_interface_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequestArray.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteAction.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)

### Generating Services
_generate_srv_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskResult.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/BasicSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PickPlaceSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_eus(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PauseSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
)

### Generating Module File
_generate_module_eus(task_planner_interface_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(task_planner_interface_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(task_planner_interface_msgs_generate_messages task_planner_interface_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequestArray.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteAction.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskResult.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskType.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/BasicSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PickPlaceSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PauseSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_eus _task_planner_interface_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(task_planner_interface_msgs_geneus)
add_dependencies(task_planner_interface_msgs_geneus task_planner_interface_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS task_planner_interface_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequestArray.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteAction.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)

### Generating Services
_generate_srv_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskResult.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/BasicSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PickPlaceSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_lisp(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PauseSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
)

### Generating Module File
_generate_module_lisp(task_planner_interface_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(task_planner_interface_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(task_planner_interface_msgs_generate_messages task_planner_interface_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequestArray.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteAction.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskResult.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskType.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/BasicSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PickPlaceSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PauseSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_lisp _task_planner_interface_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(task_planner_interface_msgs_genlisp)
add_dependencies(task_planner_interface_msgs_genlisp task_planner_interface_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS task_planner_interface_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequestArray.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteAction.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)

### Generating Services
_generate_srv_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskResult.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/BasicSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PickPlaceSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_nodejs(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PauseSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
)

### Generating Module File
_generate_module_nodejs(task_planner_interface_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(task_planner_interface_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(task_planner_interface_msgs_generate_messages task_planner_interface_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequestArray.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteAction.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskResult.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskType.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/BasicSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PickPlaceSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PauseSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_nodejs _task_planner_interface_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(task_planner_interface_msgs_gennodejs)
add_dependencies(task_planner_interface_msgs_gennodejs task_planner_interface_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS task_planner_interface_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequestArray.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteAction.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_msg_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)

### Generating Services
_generate_srv_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskResult.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/BasicSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PickPlaceSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)
_generate_srv_py(task_planner_interface_msgs
  "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PauseSkill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
)

### Generating Module File
_generate_module_py(task_planner_interface_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(task_planner_interface_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(task_planner_interface_msgs_generate_messages task_planner_interface_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequest.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/msg/MotionTaskExecutionRequestArray.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteAction.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionGoal.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionResult.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteActionFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteGoal.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteResult.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/build/devel/share/task_planner_interface_msgs/msg/TaskExecuteFeedback.msg" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskResult.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/TaskType.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/BasicSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PickPlaceSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_interface_msgs/srv/PauseSkill.srv" NAME_WE)
add_dependencies(task_planner_interface_msgs_generate_messages_py _task_planner_interface_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(task_planner_interface_msgs_genpy)
add_dependencies(task_planner_interface_msgs_genpy task_planner_interface_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS task_planner_interface_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_planner_interface_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(task_planner_interface_msgs_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(task_planner_interface_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_planner_interface_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(task_planner_interface_msgs_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(task_planner_interface_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_planner_interface_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(task_planner_interface_msgs_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(task_planner_interface_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_planner_interface_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(task_planner_interface_msgs_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(task_planner_interface_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_planner_interface_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(task_planner_interface_msgs_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(task_planner_interface_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
