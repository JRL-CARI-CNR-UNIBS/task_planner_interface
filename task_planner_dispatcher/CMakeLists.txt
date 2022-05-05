cmake_minimum_required(VERSION 3.0.2)
project(task_planner_dispatcher)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  subscription_notifier
  task_planner_interface_msgs
)

catkin_package(
 # INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs subscription_notifier task_planner_interface_msgs
  DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})


add_executable(dispatcher_single src/dispatcher_single.cpp)
add_dependencies(dispatcher_single ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dispatcher_single
   ${catkin_LIBRARIES}
   yaml-cpp
)
add_executable(dispatcher_double_sync src/dispatcher_double_sync.cpp)
add_dependencies(dispatcher_double_sync ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dispatcher_double_sync
   ${catkin_LIBRARIES}
   yaml-cpp
)
add_executable(dispatcher_double_async src/dispatcher_double_async.cpp)
add_dependencies(dispatcher_double_async ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dispatcher_double_async
   ${catkin_LIBRARIES}
   yaml-cpp
)

FILE(GLOB_RECURSE YamlFiles "config/*.yaml")
add_custom_target(yaml SOURCES ${YamlFiles})

FILE(GLOB_RECURSE LaunchFiles "launch/*.launch")
add_custom_target(launch SOURCES ${LaunchFiles})



