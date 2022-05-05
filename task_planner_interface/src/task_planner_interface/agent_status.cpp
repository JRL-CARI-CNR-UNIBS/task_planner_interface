#include <task_planner_interface/agent_status.h>

namespace taskPlannerInterface
{

AgentStatus::AgentStatus()
{
 m_object_in_hand_id.clear();
 m_object_in_hand_type.clear();
}

void AgentStatus::setNamespace(const std::string& agent_namespace){ m_agent_namespace=agent_namespace; }

std::string AgentStatus::getNamespace() const { return m_agent_namespace; }

void AgentStatus::setObjectInHand(const std::string& id, const std::string& type)
{
  m_object_in_hand_id=id;
  m_object_in_hand_type=type;
}

std::string AgentStatus::getObjectInHandId() const { return m_object_in_hand_id; }

std::string AgentStatus::getObjectInHandType() const { return m_object_in_hand_type; }

void AgentStatus::resetObjectInHand()
{
  m_object_in_hand_id.clear();
  m_object_in_hand_type.clear();
}

} //end namespace
