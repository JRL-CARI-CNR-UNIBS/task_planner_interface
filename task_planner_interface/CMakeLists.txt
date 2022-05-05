cmake_minimum_required(VERSION 2.8.3)
project(task_planner_interface)
add_compile_options(-std=c++17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++11 -fstrict-aliasing -pedantic-errors -pedantic -Wno-deprecated-declarations -Wno-unused-variable")

find_package(catkin REQUIRED COMPONENTS
  manipulation_msgs
  object_loader
  roscpp
  std_msgs
  subscription_notifier
  task_planner_interface_msgs
  mongocpp_easy_driver
)

# 3rd party tools
find_package(libmongocxx)
if (NOT libmongocxx_FOUND)
  message("mongocxx non found. Follow the instruction:")
  message("sudo apt install libbson-dev  libmongoc-dev")
  message("curl -OL https://github.com/mongodb/mongo-cxx-driver/archive/r3.1.4.tar.gz")
  message("tar -xvf r3.1.4.tar.gz")
  message("cd mongo-cxx-driver-r3.1.4/build")
  message("cmake . .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local")
  message("sudo cmake --build . --target EP_mnmlstc_core")
  message("cmake --build .")
  message("sudo cmake --build . --target install")
  message(FATAL_ERROR "follow the instruction (see lines above) after that recompile")
endif()

find_package(libbsoncxx REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS manipulation_msgs object_loader roscpp std_msgs subscription_notifier task_planner_interface_msgs mongocpp_easy_driver
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBMONGOCXX_INCLUDE_DIRS}
  ${LIBBSONCXX_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/agent_status.cpp
  src/${PROJECT_NAME}/generic_skill.cpp
  src/${PROJECT_NAME}/pick_skill.cpp
  src/${PROJECT_NAME}/place_skill.cpp
  src/${PROJECT_NAME}/pickplace_skill.cpp
  src/${PROJECT_NAME}/go_to_skill.cpp
  src/${PROJECT_NAME}/task_executor.cpp
  src/${PROJECT_NAME}/human_executor.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} ${LIBMONGOCXX_LIBRARIES} ${LIBBSONCXX_LIBRARIES})

add_executable(task_executor_node src/task_executor_node.cpp)
add_dependencies(task_executor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_executor_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   ${LIBMONGOCXX_LIBRARIES}
   ${LIBBSONCXX_LIBRARIES}
)

add_executable(sequence_executor_node src/sequence_executor_node.cpp)
add_dependencies(sequence_executor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sequence_executor_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   ${LIBMONGOCXX_LIBRARIES}
   ${LIBBSONCXX_LIBRARIES}
)

add_executable(human_executor_node src/human_executor_node.cpp)
add_dependencies(human_executor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(human_executor_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   ${LIBMONGOCXX_LIBRARIES}
   ${LIBBSONCXX_LIBRARIES}
)

add_executable(human_node src/human_node.cpp)
add_dependencies(human_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(human_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   ${LIBMONGOCXX_LIBRARIES}
   ${LIBBSONCXX_LIBRARIES}
)


add_executable(test_mongo_node src/test_mongo_node.cpp)
add_dependencies(test_mongo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_mongo_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   ${LIBMONGOCXX_LIBRARIES}
   ${LIBBSONCXX_LIBRARIES}
)

add_executable(syncronization_node src/syncronization_node.cpp)
add_dependencies(syncronization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(syncronization_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
)

FILE(GLOB_RECURSE YamlFiles "config/*.yaml")
add_custom_target(yaml SOURCES ${YamlFiles})

FILE(GLOB_RECURSE LaunchFiles "launch/*.launch")
add_custom_target(launch SOURCES ${LaunchFiles})


## Directory with the source code
#add_subdirectory(src)
