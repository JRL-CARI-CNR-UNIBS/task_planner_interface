cmake_minimum_required(VERSION 3.0.2)
project(mongocpp_easy_driver)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
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
  CATKIN_DEPENDS roscpp
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBMONGOCXX_INCLUDE_DIRS}
  ${LIBBSONCXX_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/mongocpp_easy_driver.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} ${LIBMONGOCXX_LIBRARIES} ${LIBBSONCXX_LIBRARIES})

add_executable(test_mongo_driver src/test_mongo_driver.cpp)
add_dependencies(test_mongo_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_mongo_driver
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   ${LIBMONGOCXX_LIBRARIES}
   ${LIBBSONCXX_LIBRARIES}
)

FILE(GLOB_RECURSE YamlFiles "config/*.yaml")
add_custom_target(yaml SOURCES ${YamlFiles})

FILE(GLOB_RECURSE LaunchFiles "launch/*.launch")
add_custom_target(launch SOURCES ${LaunchFiles})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)
