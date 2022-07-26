#ifndef SWITCH_DIGITAL_H
#define SWITCH_DIGITAL_H

#include "behaviortree_cpp_v3/behavior_tree.h"

#define NUM_CASES 2


class SwitchDigital : public BT::ControlNode
{
  public:
    SwitchDigital(const std::string& name, const BT::NodeConfiguration& config)
    : ControlNode::ControlNode(name, config ),
      running_child_(-1)
    {
        setRegistrationID("SwitchDigital");
    }

    virtual ~SwitchDigital() override = default;

    void halt() override
    {
        running_child_ = -1;
        ControlNode::halt();
    }

    static BT::PortsList providedPorts()
    {
        BT::PortsList ports;
        ports.insert( BT::InputPort<bool>("variable") );
        return ports;
    }

  private:
    int running_child_;
    virtual BT::NodeStatus tick() override
    {

      if( childrenCount() != NUM_CASES)
      {
          throw BT::LogicError("Wrong number of children in SwitchNode; "
                           "must be (num_cases + default)");
      }

      bool variable;
      int child_index = 1;
      if(not getInput("variable", variable))
      {
        throw BT::LogicError("Wrong number of children in SwitchNode; "
                         "must be (num_cases + default)");
      }

      if(variable)
      {
        child_index =0;
      }


      // if another one was running earlier, halt it
      if( running_child_ != -1 && running_child_ != child_index)
      {
          haltChild(running_child_);
      }

      auto& selected_child = children_nodes_[child_index];
      BT::NodeStatus ret = selected_child->executeTick();
      if( ret == BT::NodeStatus::RUNNING )
      {
          running_child_ = child_index;
      }
      else{
          haltChildren();
          running_child_ = -1;
      }
      return ret;
    }
};





#endif // SWITCH_DIGITAL_H
