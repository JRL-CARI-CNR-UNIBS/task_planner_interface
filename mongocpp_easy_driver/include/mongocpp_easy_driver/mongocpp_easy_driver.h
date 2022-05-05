#ifndef __mongocpp_easy_driver__
#define __mongocpp_easy_driver__

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

/* for Mongo */
#include <cstdint>
#include <iostream>
#include <vector>
#include <bsoncxx/v_noabi/bsoncxx/json.hpp>
#include <mongocxx/v_noabi/mongocxx/client.hpp>
#include <mongocxx/v_noabi/mongocxx/stdx.hpp>
#include <mongocxx/v_noabi/mongocxx/uri.hpp>
#include <mongocxx/v_noabi/mongocxx/instance.hpp>

using bsoncxx::builder::stream::close_array;
using bsoncxx::builder::stream::close_document;
using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;
using bsoncxx::builder::stream::open_array;
//using bsoncxx::builder::stream::array;
using bsoncxx::builder::stream::open_document;


namespace mongocpp_easy_driver
{

class mongoEasyDriver
{
protected:
  static mongocxx::instance m_instance; // This should be done only once.
  std::shared_ptr<mongocxx::client> m_client_ptr;
  mongocxx::database m_db;
  mongocxx::collection m_collection_skills;
  mongocxx::collection m_collection_results;

public:
  mongoEasyDriver();
  mongoEasyDriver(const std::string& uri_string, const std::string& db);
  mongoEasyDriver(const std::string& db);
  void setURI(const std::string& uri_string);
  void setDB(const std::string& db);
  void deleteAll(const std::string& collection);
  bool findOne(const std::string& collection, const std::string& field, const std::string& value, bsoncxx::stdx::optional<bsoncxx::document::value>& bson_doc);
  bool findOneProjected(const std::string& collection, const std::string& field, const std::string& value, const std::string& proj_name, std::string& query_result);
  void insertOne(const std::string& collection, const bsoncxx::document::value& bson_doc);
  bool appendToBSON(const std::string& field, const std::string& value, bsoncxx::document::value& bson_doc);
  bool appendArrayToBSON(const std::string& field, const std::vector<std::string>& value, bsoncxx::document::value& bson_doc);
  void testDriver(const std::string& name);
};
typedef std::shared_ptr<mongoEasyDriver> mongoEasyDriverPtr;

}

#endif
