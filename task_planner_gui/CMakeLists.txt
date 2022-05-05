cmake_minimum_required(VERSION 3.0.2)
project(task_planner_gui)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  subscription_notifier
  task_planner_interface_msgs
)

# 3rd party tools
find_package(Qt5 COMPONENTS Widgets Qml Quick QuickControls2 Sql REQUIRED)
find_package(Qt5Charts)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs subscription_notifier message_generation message_runtime task_planner_interface_msgs
  DEPENDS
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${QtQml_INCLUDE_DIRS}
)

add_definitions(${Qt5Widgets_DEFINITIONS} ${QtQml_DEFINITIONS} ${${Qt5Quick_DEFINITIONS}})

qt5_add_resources(QT_RESOURCES qt_src/qml.qrc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(NOT DEFINED HEADERS)
    file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/qt_src/*.h)
endif()

if(NOT DEFINED SOURCES)
    file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/qt_src/*.cpp)
endif()

source_group("Header Files" FILES ${HEADERS})
source_group("Source Files" FILES ${SOURCES})

add_executable(task_interface_gui ${HEADERS} ${SOURCES} ${QT_RESOURCES})
target_link_libraries(task_interface_gui
    Qt5::Widgets
    Qt5::Qml
    Qt5::Quick
    Qt5::QuickControls2
    Qt5::Sql
    Qt5::Charts
    ${catkin_LIBRARIES}
    )



