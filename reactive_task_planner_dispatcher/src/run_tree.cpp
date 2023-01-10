#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include <reactive_task_planner_dispatcher/single_task_dispatcher.h>
#include <reactive_task_planner_dispatcher/double_task_dispatcher.h>
#include <reactive_task_planner_dispatcher/wait_agent_feedback.h>
#include <reactive_task_planner_dispatcher/there_is_new_piece.h>
#include <reactive_task_planner_dispatcher/switch_digital.h>
#include <reactive_task_planner_dispatcher/check_agent_state.h>
#include <reactive_task_planner_dispatcher/reset_new_piece.h>

class SayRuntimePort : public BT::SyncActionNode
{
  public:
  SayRuntimePort(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    auto msg = getInput<std::string>("message");
    if (!msg){
      throw BT::RuntimeError( "missing required input [message]: ", msg.error() );
    }
    std::cout << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};


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

//    BT::PortsList exchange_info = {BT::OutputPort<std::string>("exchange_info_out"),BT::InputPort<std::string>("exchange_info")};
//    factory.registerBuilder(BT::CreateManifest<SingleTaskDispatcher>("SingleTaskDispatcher", exchange_info),
//                            BT::CreateBuilder<SingleTaskDispatcher>());

    factory.registerNodeType<SingleTaskDispatcher>("SingleTaskDispatcher");

    BT::PortsList say_ports = {BT::InputPort<std::string>("message")};
    factory.registerNodeType<SayRuntimePort>("SayRuntimePort", say_ports);

    factory.registerNodeType<DoubleTaskDispatcher>("DoubleTaskDispatcher");
    factory.registerNodeType<WaitAgentFeedback>("WaitAgentFeedback");
    factory.registerNodeType<ThereIsNewPiece>("ThereIsNewPiece");
    factory.registerNodeType<SwitchDigital>("SwitchDigital");
    factory.registerNodeType<CheckAgentState>("CheckAgentState");
    factory.registerNodeType<ResetNewPiece>("ResetNewPiece");



ROS_INFO("Start running tree");
//    SingleTaskDispatcher("task");

//    factory.registerNodeType<BT::SyncActionNode>("fixture");

//    ROS_INFO("FixtureCheck registered");

    ROS_INFO_STREAM("Tree path:"<<tree_path);
    ROS_INFO("Start running tree");
    BT::Tree tree = factory.createTreeFromFile(tree_path);

    ROS_INFO("Tree created");
    BT::PublisherZMQ publisher_zmq(tree);

    tree.tickRoot();

    ROS_INFO("Tree finish");
}
