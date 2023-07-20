#ifndef __agent_status__
#define __agent_status__

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

namespace taskPlannerInterface
{

class AgentStatus
{
protected:
  std::string m_agent_namespace;
  std::string m_object_in_hand_id;
  std::string m_object_in_hand_type;
public:
  AgentStatus();
  void setNamespace(const std::string& agent_namespace);
  std::string getNamespace() const;
  void setObjectInHand(const std::string& id, const std::string& type);
  std::string getObjectInHandId() const;
  std::string getObjectInHandType() const;
  void resetAgentStatus();
};
typedef std::shared_ptr<AgentStatus> AgentStatusPtr;
typedef std::shared_ptr<AgentStatus const> AgentStatusConstPtr;
}

#endif
