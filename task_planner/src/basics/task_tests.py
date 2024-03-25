import unittest
from unittest.mock import patch, MagicMock

from statistics_utils import Statistics, AgentStats, AtomicSynergy, Synergy, AgentSynergy, TaskStatistics, TaskSynergies
from task import Task


class TestTask(unittest.TestCase):
    def setUp(self):
        self.task = Task("Task1")
        self.agent_stats = AgentStats("Agent1", Statistics(1.0, 1.0))
        self.agent_synergy = AgentSynergy("Agent1", Synergy("ParallelTask", "ParallelAgent", AtomicSynergy(1.0, 0.5)))

    def test_get_task_name(self):
        self.assertEqual(self.task.get_task_name(), "Task1")

    def test_add_agent(self):
        self.task.add_agent("Agent1")
        print(self.task)
        self.assertIn("Agent1", self.task.get_agents())

    def test_add_agent_duplicate(self):
        with patch('builtins.print') as mock_print:
            self.task.add_agent("Agent1")
            self.task.add_agent("Agent1")
            self.assertEqual(mock_print.call_count, 1)  # Ensure warning message is printed once

    def test_remove_agent(self):
        self.task.add_agent("Agent1")
        self.task.remove_agent("Agent1")
        self.assertNotIn("Agent1", self.task.get_agents())

    def test_update_agents(self):
        self.task.update_agents({"Agent1", "Agent2"})
        self.assertEqual(self.task.get_agents(), {"Agent1", "Agent2"})

    def test_add_agent_statistics(self):

        self.task.add_agent_statistics(self.agent_stats)
        # self.assertIn(self.agent_stats, self.task.get_statistics())

    def test_update_agent_statistics(self):
        self.task.add_agent("Agent1")
        self.task.update_agent_statistics(self.agent_stats)
        self.assertIn(self.agent_stats, self.task.get_statistics())

    def test_get_agent_statistics(self):
        self.task.add_agent_statistics(self.agent_stats)
        self.assertEqual(self.task.get_agent_statistics("Agent1"), {self.agent_stats})

    def test_add_synergy_agent_synergy(self):
        self.task.add_synergy(self.agent_synergy)
        self.assertIn(self.agent_synergy, self.task.get_synergies())

    def test_add_synergy_task_synergies(self):
        task_synergies = TaskSynergies("Task1", "Agent1")
        task_synergies.add_synergy(self.agent_synergy)
        self.task.add_synergy(task_synergies)
        self.assertIn(self.agent_synergy, self.task.get_synergies())

    def test_update_synergy(self):
        self.task.add_synergy(self.agent_synergy)
        self.task.update_synergy(self.agent_synergy)
        self.assertIn(self.agent_synergy, self.task.get_synergies())


if __name__ == '__main__':
    unittest.main()
