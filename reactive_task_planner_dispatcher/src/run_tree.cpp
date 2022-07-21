#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <reactive_task_planner_dispatcher/single_task_dispatcher.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_tree_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string tree_path;

    if (!pnh.getParam("tree_path",tree_path))
    {
      ROS_ERROR("tree_path not defined");
      return 0;
    }

    ROS_INFO("Start running tree");

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<SingleTaskDispatcher>("SingleTaskDispatcher");
//    SingleTaskDispatcher("task");

//    factory.registerNodeType<BT::SyncActionNode>("fixture");

//    ROS_INFO("FixtureCheck registered");

    ROS_INFO_STREAM(tree_path);
    BT::Tree tree = factory.createTreeFromFile(tree_path);

    ROS_INFO("Tree created");

    tree.tickRoot();

    ROS_INFO("Tree finish");
}
