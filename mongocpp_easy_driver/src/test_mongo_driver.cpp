#include <ros/ros.h>

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
  ros::init(argc, argv, "test_mongo_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string uri_name = "mongodb://localhost:27017";
  std::string db_name = "db_test";
  std::string coll_name = "coll_test";

  mongocxx::uri uri(uri_name);
  mongocxx::client client(uri);
  mongocxx::database db = client[db_name];
  mongocxx::collection coll = db[coll_name];
  auto builder = bsoncxx::builder::stream::document{};
  bsoncxx::document::value doc_value = builder
    << "name"  << "test"
    << "value" << "one"
    << bsoncxx::builder::stream::finalize;

  bsoncxx::document::view view = doc_value.view();
  bsoncxx::stdx::optional<mongocxx::result::insert_one> result =  coll.insert_one(view);



  mongocpp_easy_driver::mongoEasyDriver mongo_driver(uri_name, db_name);
  mongo_driver.insertOne(coll_name,doc_value);

  bsoncxx::stdx::optional<bsoncxx::document::value> bson_doc;
  mongo_driver.findOne(coll_name,"name", "test", bson_doc);
  std::string tmp;
  bsoncxx::document::element ele1{bson_doc->view()["name"]};
  tmp = ele1.get_utf8().value.to_string();
  std::cout << "name: " << tmp.c_str() << "\n";

  bsoncxx::document::element ele2{bson_doc->view()["value"]};
  tmp = ele2.get_utf8().value.to_string();
  std::cout << "value: " << tmp.c_str() << "\n";

  mongo_driver.findOneProjected(coll_name,"name","test","value",tmp);
  std::cout << "Projected: value = " << tmp.c_str() << "\n";

  mongo_driver.appendToBSON("new field","new value",doc_value);
  mongo_driver.insertOne(coll_name,doc_value);

  ROS_INFO("Exit normally.");
  return 0;
  
}


