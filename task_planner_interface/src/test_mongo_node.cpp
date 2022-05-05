#include <ros/ros.h>
#include <task_planner_interface_msgs/MotionTaskExecutionFeedback.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequest.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequestArray.h>
#include <task_planner_interface/generic_skill.h>

//#include <subscription_notifier/subscription_notifier.h>
//#include <fstream>
//#include <sstream>
//#include <yaml-cpp/yaml.h>
//#include <string>
#include <cstdint>
#include <iostream>
#include <vector>
#include <bsoncxx/v_noabi/bsoncxx/json.hpp>
#include <mongocxx/v_noabi/mongocxx/client.hpp>
#include <mongocxx/v_noabi/mongocxx/stdx.hpp>
#include <mongocxx/v_noabi/mongocxx/uri.hpp>
#include <mongocxx/v_noabi/mongocxx/instance.hpp>
#include <mongocpp_easy_driver/mongocpp_easy_driver.h>

using bsoncxx::builder::stream::close_array;
using bsoncxx::builder::stream::close_document;
using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;
using bsoncxx::builder::stream::open_array;
using bsoncxx::builder::stream::open_document;

mongocxx::instance mongocpp_easy_driver::mongoEasyDriver::m_instance{};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_mongo_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string uri_name = "mongodb://localhost:27017";
  std::string db_name = "db_test";
  std::string coll_name = "coll_test";
  std::string coll2_name = "coll2_test";


  /* Create a BSON document and insert in the mongoDB */
  auto builder = bsoncxx::builder::stream::document{};
  bsoncxx::document::value doc_value = builder
    << "name"  << "test name"
    << "type" << "test type"
    << "description" << "test description"
    << bsoncxx::builder::stream::finalize;

  bsoncxx::document::view view = doc_value.view();

  mongocpp_easy_driver::mongoEasyDriver mongo_driver(uri_name, db_name);
  mongo_driver.deleteAll(coll_name);
  mongo_driver.deleteAll(coll2_name);

  mongo_driver.insertOne(coll_name,doc_value);

  /* Find the task property document */
  bsoncxx::stdx::optional<bsoncxx::document::value> bson_doc;
  mongo_driver.findOne(coll_name,"name", "test name", bson_doc);

  /* Save it to a generic skill and get the results */
  taskPlannerInterface::skills::GenericSkillPtr skill;
  skill.reset(new taskPlannerInterface::skills::GenericSkill);
  std::string tmp = "m";
  skill->init(tmp);
  skill->setPropertiesFromBSON(bson_doc);
  skill->print();
  auto doc_results = skill->getResultsAsBSON();

  /* Insert result into different collection */
  mongo_driver.appendToBSON("recipe","test recipe name",doc_results);
  mongo_driver.insertOne(coll2_name,doc_results);

  ROS_INFO("Exit normally.");
  return 0;

}


