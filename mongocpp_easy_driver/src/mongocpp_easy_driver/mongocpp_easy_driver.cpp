#include <mongocpp_easy_driver/mongocpp_easy_driver.h>

#include <mongocpp_easy_driver/mongocpp_easy_driver.h>

namespace mongocpp_easy_driver
{

mongoEasyDriver::mongoEasyDriver(){}

mongoEasyDriver::mongoEasyDriver(const std::string& uri_string, const std::string& db)
{
  mongocxx::uri uri(uri_string);
  m_client_ptr.reset(new mongocxx::client(uri));
  m_db = m_client_ptr->database(db);
}

mongoEasyDriver::mongoEasyDriver(const std::string& db)
{
  mongocxx::uri uri("mongodb://localhost:27017");
  m_client_ptr.reset(new mongocxx::client(uri));
  m_db = m_client_ptr->database(db);
}

void mongoEasyDriver::setURI(const std::string& uri_string)
{
  mongocxx::uri uri(uri_string);
  m_client_ptr.reset(new mongocxx::client(uri));
  std::cout << "URI of mongoDB updated. Make sure to reset DB names. New URI: " << uri_string.c_str();
}

void mongoEasyDriver::setDB(const std::string& db){ m_db = m_client_ptr->database(db); }

void mongoEasyDriver::deleteAll(const std::string& collection)
{
  mongocxx::collection coll = m_db[collection];
  auto filter = document{}<< finalize;
  coll.delete_many(filter.view());
}


bool mongoEasyDriver::findOne(const std::string& collection, const std::string& field, const std::string& value, bsoncxx::stdx::optional<bsoncxx::document::value>& bson_doc)
{
  mongocxx::collection coll = m_db[collection];
  /* Create query filter*/
  auto filter = document{} << field << value << finalize;
  /* Query */
  auto result = coll.find_one(filter.view());
  if(result)
  {
    bson_doc = result;
    return true;
  }
  else
    return false;
}

bool mongoEasyDriver::findOneProjected(const std::string& collection, const std::string& field, const std::string& value, const std::string& proj_name, std::string& query_result)
{
  mongocxx::collection coll = m_db[collection];
  /* Create query filter*/
  auto filter = document{} << field << value << finalize;
  /* Set projection opt to get single field */
  mongocxx::options::find opts{};
  opts.projection(document{} << proj_name << 1 << finalize);
  /* Query */
  auto result = coll.find_one(filter.view(), opts);
  if(result) {
    bsoncxx::document::element ele{result->view()[proj_name]};
    query_result = ele.get_utf8().value.to_string();
    return true;
  }
  else
    return false;
}

void mongoEasyDriver::insertOne(const std::string& collection, const bsoncxx::document::value& bson_doc)
{
  mongocxx::collection coll = m_db[collection];
  bsoncxx::document::view view = bson_doc.view();
  bsoncxx::stdx::optional<mongocxx::result::insert_one> result =  coll.insert_one(view);
}

bool mongoEasyDriver::appendToBSON(const std::string& field, const std::string& value, bsoncxx::document::value& bson_doc)
{
  auto builder = bsoncxx::builder::stream::document{};
  bsoncxx::document::value doc_value_append = builder
    << field << value
    << bsoncxx::builder::stream::finalize;

  using bsoncxx::builder::concatenate;
  auto builder2 = bsoncxx::builder::stream::document{};
  bsoncxx::document::value tmp = builder2
                                        << bsoncxx::builder::concatenate(bson_doc.view())
                                        << bsoncxx::builder::concatenate(doc_value_append.view())
                                        << bsoncxx::builder::stream::finalize;
  bson_doc = tmp;

}

bool mongoEasyDriver::appendArrayToBSON(const std::string& field, const std::vector<std::string>& value, bsoncxx::document::value& bson_doc)
{
  auto array_builder = bsoncxx::builder::basic::array{};
  for (unsigned int idx=0;idx<value.size();idx++)
    array_builder.append(value.at(idx));
  auto builder = bsoncxx::builder::stream::document{};
  bsoncxx::document::value doc_value_append = builder
    << field << array_builder
    << bsoncxx::builder::stream::finalize;

  using bsoncxx::builder::concatenate;
  auto builder2 = bsoncxx::builder::stream::document{};
  bsoncxx::document::value tmp = builder2
                                        << bsoncxx::builder::concatenate(bson_doc.view())
                                        << bsoncxx::builder::concatenate(doc_value_append.view())
                                        << bsoncxx::builder::stream::finalize;
  bson_doc = tmp;
}



//bool mongoEasyDriver::getSkillType(const std::string& id, std::string& skill_type)
//{
//  /* Create query filter*/
//  auto filter = document{} << "name" << id << finalize;
//  /* Set projection opt to get single field */
//  mongocxx::options::find opts{};
//  opts.projection(document{} << "type" << 1 << finalize);
//  /* Query */
//  auto result = m_collection_skills.find_one(filter.view(), opts);
//  if(result) {
//    bsoncxx::document::element ele{result->view()["type"]};
//    skill_type = ele.get_utf8().value.to_string();
//    return true;
//  }
//  else
//    return false;
//}

//void mongoEasyDriver::saveSkillResults(const taskPlannerInterface::skills::GenericSkillPtr skill, const std::string& group_name)
//{
//  int outcome=0;
//  double duration_planned=0.0;
//  double duration_real=0.0;
//  double planning_time=0.0;
//  double path_length=0.0;
//  skill->getResults(outcome,duration_real,duration_planned,planning_time,path_length);
//  std::vector<std::string> concurrent_tasks = skill->getConcurrentTasks();
//  auto array_builder = bsoncxx::builder::basic::array{};
//  for (unsigned int idx=0;idx<concurrent_tasks.size();idx++)
//    array_builder.append(concurrent_tasks.at(idx));

//  auto builder = bsoncxx::builder::stream::document{};
//  bsoncxx::document::value doc_value = builder
//    << "name" << skill->getName()
//    << "type" << skill->getType()
//    << "outcome" << outcome
//    << "duration_planned" << duration_planned
//    << "duration_real" << duration_real
//    << "planning_time" << planning_time
//    << "path_length" << path_length
//    << "concurrent_tasks" << array_builder
//    << "agent" << group_name
//    << "recipe_name" << "none"
//    << "date" <<bsoncxx::types::b_date(std::chrono::system_clock::now())
//  << bsoncxx::builder::stream::finalize;
//  bsoncxx::document::view view = doc_value.view();
//  bsoncxx::stdx::optional<mongocxx::result::insert_one> result =  m_collection_results.insert_one(view);
//}

//void mongoEasyDriver::saveSkillResults(const taskPlannerInterface::skills::GenericSkillPtr skill, const std::string& group_name, const std::string& recipe_name)
//{
//  int outcome=0;
//  double duration_planned=0.0;
//  double duration_real=0.0;
//  double planning_time=0.0;
//  double path_length=0.0;
//  skill->getResults(outcome,duration_real,duration_planned,planning_time,path_length);
//  std::vector<std::string> concurrent_tasks = skill->getConcurrentTasks();
//  auto array_builder = bsoncxx::builder::basic::array{};
//  for (unsigned int idx=0;idx<concurrent_tasks.size();idx++)
//    array_builder.append(concurrent_tasks.at(idx));

//  auto builder = bsoncxx::builder::stream::document{};
//  bsoncxx::document::value doc_value = builder
//    << "name" << skill->getName()
//    << "type" << skill->getType()
//    << "outcome" << outcome
//    << "duration_planned" << duration_planned
//    << "duration_real" << duration_real
//    << "planning_time" << planning_time
//    << "path_length" << path_length
//    << "concurrent_tasks" << array_builder
//    << "agent" << group_name
//    << "recipe_name" << recipe_name
//    << "date" <<bsoncxx::types::b_date(std::chrono::system_clock::now())
//  << bsoncxx::builder::stream::finalize;
//  bsoncxx::document::view view = doc_value.view();
//  bsoncxx::stdx::optional<mongocxx::result::insert_one> result =  m_collection_results.insert_one(view);

//}

void mongoEasyDriver::testDriver(const std::string& var)
{
  auto builder = bsoncxx::builder::stream::document{};
  bsoncxx::document::value doc_value = builder
    << "name" << var
    << "vector" << bsoncxx::builder::stream::open_array
      << "1" << "2" << "3"
    << close_array
    << "list" << bsoncxx::builder::stream::open_document
      << "x" << 203
      << "y" << 102
    << bsoncxx::builder::stream::close_document
    << bsoncxx::builder::stream::finalize;
    bsoncxx::document::view view = doc_value.view();
    bsoncxx::stdx::optional<mongocxx::result::insert_one> result =  m_collection_results.insert_one(view);
}

//bool mongoEasyDriver::queryByName(const std::string& id, bsoncxx::stdx::optional<bsoncxx::document::value>& bson_doc)
//{
//  auto filter = document{} << "name" << id << finalize;
//  bsoncxx::stdx::optional<bsoncxx::document::value> result = m_collection_skills.find_one(filter.view());
//  if(result) {
//    bson_doc = result;
//    return true;
//  }
//  else
//    return false;
//}

} //end namespace
