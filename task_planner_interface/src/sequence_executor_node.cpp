#include <tuple>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <manipulation_msgs/PickObjectsAction.h>
#include <manipulation_msgs/PlaceObjectsAction.h>
#include <manipulation_msgs/GoToAction.h>
#include <manipulation_msgs/RemoveObjectFromSlot.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <rosparam_utilities/rosparam_utilities.h>

#include <task_planner_interface/task_executor.h>
#include <task_planner_interface_msgs/TaskExecuteAction.h>


class SequenceExecuteAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<task_planner_interface_msgs::TaskExecuteAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  std::string group_name_;

  ros::ServiceClient m_skill_type_client;
  ros::ServiceClient m_skill_properties_client;



  std::shared_ptr<actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction>>  pick_ac;
  std::shared_ptr<actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction>> place_ac;
  std::shared_ptr<actionlib::SimpleActionClient<manipulation_msgs::GoToAction>> go_to_ac;

  ros::ServiceClient remove_object_from_slot_clnt;

public:

  SequenceExecuteAction(std::string group_name, std::string name) :
    as_(nh_, name, boost::bind(&SequenceExecuteAction::executeCB, this, _1), false),
    action_name_(name),
    group_name_(group_name)
  {
    ros::AsyncSpinner spinner(4);
    spinner.start();

    pick_ac.reset( new actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction>("/inbound_pick_server/"+group_name+"/pick"));
    place_ac.reset( new actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction>("/outbound_place_server/"+group_name+"/place"));
    go_to_ac.reset(new actionlib::SimpleActionClient<manipulation_msgs::GoToAction>("/go_to_location_server/"+group_name+"/go_to"));

    ROS_INFO("Waiting for pick server");
    pick_ac->waitForServer();
    ROS_INFO("Connection ok");
    ROS_INFO("Waiting for place server");
    place_ac->waitForServer();
    ROS_INFO("Connection ok");
    ROS_INFO("Waiting for goto server");
    go_to_ac->waitForServer();
    ROS_INFO("Connection ok");

    remove_object_from_slot_clnt = nh_.serviceClient<manipulation_msgs::RemoveObjectFromSlot>("/outbound_place_server/remove_obj_from_slot");
    remove_object_from_slot_clnt.waitForExistence();

    /* Server clients for mongo*/
    m_skill_type_client = nh_.serviceClient <task_planner_interface_msgs::TaskType>("mongo_handler/check_task_type");
    m_skill_properties_client = nh_.serviceClient <task_planner_interface_msgs::BasicSkill>("mongo_handler/get_task_properties");

    as_.start();
  }

  ~SequenceExecuteAction(void)
  {
  }

  bool checkSkillType(const std::string name, std::string& skill_type)
  {
    task_planner_interface_msgs::TaskType srv;
    srv.request.name = name;

    if(m_skill_type_client.call(srv))
    {
        if(srv.response.exist)
        {
            ROS_INFO("Task type: %s", srv.response.type.c_str());
            skill_type=srv.response.type;
            return true;
        }
        else
        {
            ROS_ERROR("Task type does not exist");
        }

    }
    else
    {
        ROS_ERROR("Failed to call service check_type");
    }
    return false;   // Both for service fail or task doesn't exist
  }


  void executeCB(const task_planner_interface_msgs::TaskExecuteGoalConstPtr &goal)
  {
    std::vector<std::tuple<std::string, // action
                std::vector<std::string>, // description
                std::string, // approach_loc_ctrl_id
                std::string, // to_loc_ctrl_id
                std::string, // leave_loc_ctrl_id
                std::string, // tool_id
                std::string, // job_exec_name
                std::string, // property_pre_exec_id
                std::string, // property_exec_id
                std::string>> // property_post_exec_id
                recipe;

    std::string skill_type;
    try
    {
      if(checkSkillType(goal->name,skill_type))
        ROS_INFO("Skill received");
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR("Unable to find field 'goal' in mongoDB", ex.what());
      return;
    }

    std::string skill_recipe;

    task_planner_interface_msgs::BasicSkill srv;
    srv.request.name = goal->name;

    if(m_skill_properties_client.call(srv))
    {
        if(!srv.response.error)   // If no error
        {
            ROS_INFO("Task properties retrieved correctly");
            skill_recipe = srv.response.goal[0];
        }
        else
        {
            ROS_ERROR("Task does not exist");
        }
    }
    else
    {
        ROS_ERROR("Rosservice call for retrive properties failed");
    }



    XmlRpc::XmlRpcValue param;
    if (!nh_.getParam(skill_recipe + "/recipe",param))
    {
      ROS_INFO_STREAM("Recipe not found: " << skill_recipe + "/recipe");

      ROS_ERROR("Recipe not found: %s",goal->name.c_str());
      return;
    }

    if (param.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Recipe is not a list" );
      return;
    }

    for(int i=0; i<param.size(); i++)
    {
      std::tuple< std::string, // action
                  std::vector<std::string>, // description
                  std::string, // approach_loc_ctrl_id
                  std::string, // to_loc_ctrl_id
                  std::string, // leave_loc_ctrl_id
                  std::string, // job_exec_name
                  std::string, // tool_id
                  std::string, // property_pre_exec_id
                  std::string, // property_exec_id
                  std::string> // property_post_exec_id
                  sigle_skill;
      XmlRpc::XmlRpcValue config = param[i];
      if( config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_WARN("The element #%zu is not a struct", i);
        continue;
      }

      if( !config.hasMember("action") )
      {
        ROS_WARN("The element #%zu has not the field 'action'", i);
        continue;
      }
      std::string action = rosparam_utilities::toString(config["action"]);

      if( !config.hasMember("description") )
      {
        ROS_WARN("The element #%zu has not the field 'description'", i);
        continue;
      }

      std::string what;
      std::vector<std::string> description;
      if( !rosparam_utilities::getParam(config,"description",description,what) )
      {
        ROS_WARN("The element #%zu has not the field 'description'", i);
        continue;
      }

      std::string approach_loc_ctrl_id;
      if( !config.hasMember("approach_loc_ctrl_id") )
      {
        ROS_WARN("The element #%zu has not the field 'approach_loc_ctrl_id'", i);
        approach_loc_ctrl_id = "";
      }
      else
      {
        approach_loc_ctrl_id = rosparam_utilities::toString(config["approach_loc_ctrl_id"]);
      }

      std::string to_loc_ctrl_id;
      if( !config.hasMember("to_loc_ctrl_id") )
      {
        ROS_WARN("The element #%zu has not the field 'to_loc_ctrl_id'", i);
        to_loc_ctrl_id = "";
      }
      else
      {
        to_loc_ctrl_id = rosparam_utilities::toString(config["to_loc_ctrl_id"]);
      }

      std::string leave_loc_ctrl_id;
      if( !config.hasMember("leave_loc_ctrl_id") )
      {
        ROS_WARN("The element #%zu has not the field 'leave_loc_ctrl_id'", i);
        leave_loc_ctrl_id = "";
      }
      else
      {
        leave_loc_ctrl_id = rosparam_utilities::toString(config["leave_loc_ctrl_id"]);
      }

      std::string job_exec_name;
      if( !config.hasMember("job_exec_name") )
      {
        ROS_WARN("The element #%zu has not the field 'job_exec_name'", i);
        job_exec_name = "";
      }
      else
      {
        job_exec_name = rosparam_utilities::toString(config["job_exec_name"]);
      }

      std::string tool_id;
      if( !config.hasMember("tool_id") )
      {
        ROS_WARN("The element #%zu has not the field 'tool_id'", i);
        tool_id = "";
      }
      else
      {
        tool_id = rosparam_utilities::toString(config["tool_id"]);
      }

      std::string property_pre_exec_id;
      if( !config.hasMember("property_pre_exec_id") )
      {
        ROS_WARN("The element #%zu has not the field 'property_pre_exec_id'", i);
        property_pre_exec_id = "";
      }
      else
      {
        property_pre_exec_id = rosparam_utilities::toString(config["property_pre_exec_id"]);
      }

      std::string property_exec_id;
      if( !config.hasMember("property_exec_id") )
      {
        ROS_WARN("The element #%zu has not the field 'property_exec_id'", i);
        property_exec_id = "";
      }
      else
      {
        property_exec_id = rosparam_utilities::toString(config["property_exec_id"]);
      }

      std::string property_post_exec_id;
      if( !config.hasMember("property_post_exec_id") )
      {
        ROS_WARN("The element #%zu has not the field 'property_post_exec_id'", i);
        property_post_exec_id = "";
      }
      else
      {
        property_post_exec_id = rosparam_utilities::toString(config["property_post_exec_id"]);
      }

      sigle_skill = make_tuple( action,
                                description,
                                approach_loc_ctrl_id,
                                to_loc_ctrl_id,
                                leave_loc_ctrl_id,
                                job_exec_name,
                                tool_id,
                                property_pre_exec_id,
                                property_exec_id,
                                property_post_exec_id);

      recipe.push_back(sigle_skill);
    }

    manipulation_msgs::RemoveObjectFromSlot remove_object_from_slot;

    for (const std::tuple<std::string,
                          std::vector<std::string>,
                          std::string,
                          std::string,
                          std::string,
                          std::string,
                          std::string,
                          std::string,
                          std::string,
                          std::string>& skill: recipe)
    {
      ROS_INFO("skill -> %s",std::get<0>(skill).c_str());

      if (std::get<0>(skill).compare("pick")==0)
      {
        manipulation_msgs::PickObjectsGoal pick_goal;

        for (const std::string& object_type: std::get<1>(skill))
        {
          ROS_INFO("[Group %s] Goal: pick object %s",group_name_.c_str(),object_type.c_str());
          pick_goal.object_types.push_back(object_type);
        }

        pick_goal.approach_loc_ctrl_id = std::get<2>(skill);
        pick_goal.to_loc_ctrl_id = std::get<3>(skill);
        pick_goal.leave_loc_ctrl_id = std::get<4>(skill);

        pick_goal.job_exec_name = std::get<5>(skill);;
        pick_goal.tool_id = std::get<6>(skill);
        pick_goal.property_pre_exec_id = std::get<7>(skill);
        pick_goal.property_exec_id = std::get<8>(skill);
        pick_goal.property_post_exec_id = std::get<9>(skill);

        pick_ac->sendGoalAndWait(pick_goal);

        if (pick_ac->getResult()->result < 0)
        {
          ROS_ERROR("[Group %s] unable to pick -> object type = %s",group_name_.c_str(),pick_ac->getResult()->object_name.c_str());
          as_.setAborted();
          return;
        }
        ROS_INFO("[Group %s] well done! I picked it, name = %s",group_name_.c_str(),pick_ac->getResult()->object_name.c_str());
      }
      else if (std::get<0>(skill).compare("place")==0)
      {
        manipulation_msgs::PlaceObjectsGoal place_goal;

        for (const std::string& slot: std::get<1>(skill))
        {
          ROS_INFO("[Group %s] Goal: place object %s in slot %s",group_name_.c_str(),place_goal.object_name.c_str(), slot.c_str());
          place_goal.slots_group_names.push_back(slot);
        }

        if (!pick_ac->getResult()->object_name.empty())
          place_goal.object_name = pick_ac->getResult()->object_name;
        else
        {
          ROS_ERROR("[Group %s] no object name = %s",group_name_.c_str(),pick_ac->getResult()->object_name.c_str());
          as_.setAborted();
          return;
        }

        place_goal.approach_loc_ctrl_id = std::get<2>(skill);
        place_goal.to_loc_ctrl_id = std::get<3>(skill);
        place_goal.leave_loc_ctrl_id = std::get<4>(skill);

        place_goal.job_exec_name = std::get<5>(skill);;
        place_goal.tool_id = std::get<6>(skill);
        place_goal.property_pre_exec_id = std::get<7>(skill);
        place_goal.property_exec_id = std::get<8>(skill);
        place_goal.property_post_exec_id = std::get<9>(skill);

        place_ac->sendGoalAndWait(place_goal);

        if (place_ac->getResult()->result < 0)
        {
          ROS_ERROR("[Group %s] unable to place -> object name = %s",group_name_.c_str(), place_goal.object_name.c_str());
          as_.setAborted();
          return;
        }

        // Simulate slot cleaning
        remove_object_from_slot.request.object_to_remove_name = pick_ac->getResult()->object_name;
        remove_object_from_slot.request.slot_name = place_ac->getResult()->slot_name;

        if (!remove_object_from_slot_clnt.call(remove_object_from_slot))
        {
          ROS_ERROR("Unespected error calling %s service",remove_object_from_slot_clnt.getService().c_str());
          as_.setAborted();
          return;
        }

        ROS_INFO("[Group %s] well done! ",group_name_.c_str());

      }
      else if (std::get<0>(skill).compare("goto")==0)
      {
        manipulation_msgs::GoToGoal go_to_goal;

        if (std::get<1>(skill).size() > 0)
          ROS_INFO("[Group %s] Goal: Go to %s",group_name_.c_str(),std::get<1>(skill).at(0).c_str());

        go_to_goal.location_names.push_back( std::get<1>(skill).at(0) );

        go_to_goal.to_loc_ctrl_id = std::get<3>(skill);

        go_to_goal.job_exec_name = std::get<5>(skill);
        go_to_goal.tool_id = std::get<6>(skill);
        go_to_goal.property_exec_id = std::get<8>(skill);

        go_to_ac->sendGoalAndWait(go_to_goal);

        if (go_to_ac->getResult()->result < 0)
        {
          ROS_ERROR("[Group %s] unable to go to -> location name = %s",group_name_.c_str(), go_to_goal.location_names.at(0).c_str());
          ROS_ERROR("Error: %d", go_to_ac->getResult()->result);

          as_.setAborted();
          return;
        }
        ROS_INFO("[Group %s] well done! ",group_name_.c_str());
      }
      else
      {
        ROS_ERROR("unable to execute action %s",std::get<0>(skill).c_str());
        as_.setAborted();
        return;
      }

      ros::Duration(0.1).sleep();
    }

    task_planner_interface_msgs::TaskExecuteResult result;
    result.outcome = 1;
    result.type = skill_type;
    result.agent = group_name_;

    as_.setSucceeded(result);

  }


};

int main(int argc, char **argv)
{
  std::string action_name = "/sequence_execute";

  ros::init(argc, argv, "sequence_executor");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string topic_request_from_planner;
  if (!pnh.getParam("topic_request_from_planner",topic_request_from_planner))
  {
    ROS_ERROR("topic_request_from_planner not defined");
    return 0;
  }



  std::string group_name="";
  if (!pnh.getParam("group_name",group_name))
  {
    ROS_ERROR_STREAM(group_name << ": "<<pnh.getNamespace()<<"/group_name not defined");
    return 0;
  }


  SequenceExecuteAction sequence_executor(group_name, topic_request_from_planner+action_name);

  ros::spin();
  return 0;

}
