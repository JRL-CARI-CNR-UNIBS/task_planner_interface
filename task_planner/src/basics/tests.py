import unittest
from task import Task, AgentStats, AgentSynergy


class TestTask(unittest.TestCase):

    def setUp(self):
        self.task = Task(task_name="Test Task")
        self.agent_stats = AgentStats(agent_name="Agent1", stats_value=10)
        self.agent_synergy = AgentSynergy(main_agent_name="Agent1", parallel_agent_name="Agent2", synergy_value=20)

    def test_add_agent_statistics(self):
        self.task.statistics = set()
        self.task.add_agent_statistics(self.agent_stats)
        self.assertIn(self.agent_stats, self.task.statistics)

    def test_update_agent_statistics(self):
        self.task.statistics = {self.agent_stats}
        self.task.update_agents({"Agent1"})
        self.task.update_agent_statistics(self.agent_stats)
        self.assertIn(self.agent_stats, self.task.statistics)

    # Add more tests for other methods...


if __name__ == '__main__':
    unittest.main()
