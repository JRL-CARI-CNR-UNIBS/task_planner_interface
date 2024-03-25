import unittest
from unittest.mock import patch, MagicMock
from statistics_utils import Statistics, AgentStats, AtomicSynergy, Synergy, AgentSynergy, TaskStatistics, TaskSynergies


class TestStatistics(unittest.TestCase):

    def test_init(self):
        with self.assertRaises(ValueError):
            Statistics(expected_duration=-1, duration_std_dev=2)  # expected_duration < 0

        with self.assertRaises(ValueError):
            Statistics(expected_duration=2, duration_std_dev=-1)  # duration_std_dev < 0

        with self.assertRaises(ValueError):
            Statistics(expected_duration=-2, duration_std_dev=-1)  # expected_duration < 0

        # Test pass with valid values
        statistics = Statistics(expected_duration=2, duration_std_dev=3)
        self.assertEqual(statistics.expected_duration, 2)
        self.assertEqual(statistics.duration_std_dev, 3)

    def test_get_expected_duration(self):
        statistics = Statistics(expected_duration=2, duration_std_dev=3)
        self.assertEqual(statistics.get_expected_duration(), 2)

    def test_get_duration_std_dev(self):
        statistics = Statistics(expected_duration=2, duration_std_dev=3)
        self.assertEqual(statistics.get_duration_std_dev(), 3)


class TestAgentStats(unittest.TestCase):

    def setUp(self):
        self.statistics = Statistics(expected_duration=2, duration_std_dev=3)
        self.agent_stats = AgentStats(agent_name="Agent1", statistics=self.statistics)

    def test_get_statistics(self):
        self.assertEqual(self.agent_stats.get_statistics(), self.statistics)

    def test_get_agent_name(self):
        self.assertEqual(self.agent_stats.get_agent_name(), "Agent1")

    def test_hash(self):
        agent_stats_1 = AgentStats(agent_name="Agent1", statistics=self.statistics)
        agent_stats_2 = AgentStats(agent_name="Agent2", statistics=self.statistics)
        statistic_3 = Statistics(expected_duration=1, duration_std_dev=1)
        agent_stats_3 = AgentStats(agent_name="Agent1", statistics=statistic_3)

        self.assertEqual(hash(agent_stats_1), hash(agent_stats_1))
        self.assertNotEqual(hash(agent_stats_1), hash(agent_stats_2))
        self.assertEqual(hash(agent_stats_1), hash(agent_stats_3))

    def test_eq(self):
        agent_stats_1 = AgentStats(agent_name="Agent1", statistics=self.statistics)
        agent_stats_2 = AgentStats(agent_name="Agent2", statistics=self.statistics)
        statistic_3 = Statistics(expected_duration=1, duration_std_dev=1)
        agent_stats_3 = AgentStats(agent_name="Agent1", statistics=statistic_3)

        self.assertEqual(agent_stats_1, agent_stats_1)
        self.assertNotEqual(agent_stats_1, agent_stats_2)
        self.assertNotEqual(agent_stats_1, None)
        self.assertEqual(agent_stats_1, agent_stats_3)

    def test_set(self):
        agent_stats_1 = AgentStats(agent_name="Agent1", statistics=self.statistics)
        agent_stats_2 = AgentStats(agent_name="Agent2", statistics=self.statistics)
        agent_stats_3 = AgentStats(agent_name="Agent3", statistics=self.statistics)

        agents_stats = set()
        agents_stats.add(agent_stats_1)
        agents_stats.add(agent_stats_2)
        agents_stats.add(agent_stats_3)
        self.assertEqual(len(agents_stats), 3)

    def test_set_equal_elements(self):
        agent_stats_1 = AgentStats(agent_name="Agent1", statistics=self.statistics)
        statistic_3 = Statistics(expected_duration=1, duration_std_dev=1)
        agent_stats_3 = AgentStats(agent_name="Agent1", statistics=statistic_3)

        agents_stats = set()
        agents_stats.add(agent_stats_1)
        agents_stats.add(agent_stats_1)
        self.assertEqual(len(agents_stats), 1)
        self.assertTrue(agent_stats_1 in agents_stats)
        self.assertTrue(agent_stats_3 in agents_stats)
        agents_stats.add(agent_stats_3)
        self.assertEqual(len(agents_stats), 1)
        agent_stat = agents_stats.pop()
        self.assertNotEqual(agent_stat.get_statistics().get_expected_duration(), 1)


class TestAtomicSynergy(unittest.TestCase):
    def test_init_positive_synergy(self):
        # Test initialization with a positive synergy value
        atomic_synergy = AtomicSynergy(10.0, 1.0)
        self.assertEqual(atomic_synergy.expected_synergy, 10.0)
        self.assertEqual(atomic_synergy.synergy_std_dev, 1.0)

    def test_init_negative_synergy(self):
        # Test initialization with a negative synergy value
        with self.assertRaises(ValueError):
            AtomicSynergy(-10.0, 1.0)

    def test_init_negative_std_dev(self):
        # Test initialization with a negative standard deviation value
        with self.assertRaises(ValueError):
            AtomicSynergy(10.0, -1.0)

    def test_init_negative(self):
        # Test initialization with both negative values
        with self.assertRaises(ValueError):
            AtomicSynergy(-10.0, -1.0)

    def test_init_none_std_dev(self):
        # Test initialization with None standard deviation
        atomic_synergy = AtomicSynergy(10.0, None)
        self.assertEqual(atomic_synergy.expected_synergy, 10.0)
        self.assertIsNone(atomic_synergy.synergy_std_dev)


class TestSynergy(unittest.TestCase):
    def setUp(self):
        self.atomic_synergy = AtomicSynergy(10.0, 1.0)
        self.synergy = Synergy("Task1", "Agent1", self.atomic_synergy)

    def test_get_parallel_task_name(self):
        # Test get_parallel_task_name method
        self.assertEqual(self.synergy.get_parallel_task_name(), "Task1")

    def test_get_parallel_agent_name(self):
        # Test get_parallel_agent_name method
        self.assertEqual(self.synergy.get_parallel_agent_name(), "Agent1")

    def test_get_synergy(self):
        # Test get_synergy method
        self.assertEqual(self.synergy.get_synergy(), self.atomic_synergy)

    def test_hash(self):
        # Test __hash__ method
        expected_hash = hash(("Task1", "Agent1"))
        synergy_different2 = Synergy("Task2", "Agent1", self.atomic_synergy)
        synergy_different3 = Synergy("Task2", "Agent1", self.atomic_synergy)
        synergy_different4 = Synergy("Task1", "Agent2", self.atomic_synergy)
        synergy_same5 = Synergy("Task1", "Agent1", self.atomic_synergy)

        self.assertEqual(hash(self.synergy), expected_hash)
        self.assertNotEqual(hash(self.synergy), hash(synergy_different2))
        self.assertNotEqual(hash(self.synergy), hash(synergy_different3))
        self.assertNotEqual(hash(self.synergy), hash(synergy_different4))
        self.assertEqual(hash(self.synergy), hash(synergy_same5))

    def test_eq(self):
        # Test __eq__ method
        same_synergy = Synergy("Task1", "Agent1", self.atomic_synergy)
        synergy_different2 = Synergy("Task2", "Agent1", self.atomic_synergy)
        synergy_different3 = Synergy("Task2", "Agent1", self.atomic_synergy)
        synergy_different4 = Synergy("Task1", "Agent2", self.atomic_synergy)

        self.assertTrue(self.synergy == same_synergy)
        self.assertFalse(self.synergy == synergy_different2)
        self.assertFalse(self.synergy == synergy_different3)
        self.assertFalse(self.synergy == synergy_different4)

        self.assertFalse(self.synergy == "Not a Synergy object")


class TestAgentSynergy(unittest.TestCase):
    def setUp(self):
        self.synergy1 = Synergy("Task1", "Agent1", None)  # TODO: Impedire il None
        self.synergy2 = Synergy("Task2", "Agent2", None)

    def test_init_different_agent_name(self):
        # Test initialization with different main and parallel agent names
        agent_synergy = AgentSynergy("MainAgent", self.synergy1)
        self.assertEqual(agent_synergy.get_main_agent(), "MainAgent")
        self.assertEqual(agent_synergy.get_parallel_agent_name(), "Agent1")
        self.assertEqual(agent_synergy.get_synergy(), self.synergy1)

    def test_init_same_agent_name(self):
        # Test initialization with same main and parallel agent names
        with self.assertRaises(ValueError) as cm:
            AgentSynergy("Agent1", self.synergy1)
        self.assertEqual(str(cm.exception), "Other agent name: (Agent1) must differ by main one: (Agent1)")

    def test_eq(self):
        # Test __eq__ method
        agent_synergy1 = AgentSynergy("MainAgent", self.synergy1)
        agent_synergy2 = AgentSynergy("MainAgent", self.synergy1)
        agent_synergy3 = AgentSynergy("MainAgent", self.synergy2)

        self.assertTrue(agent_synergy1 == agent_synergy2)
        self.assertFalse(agent_synergy1 == agent_synergy3)

    def test_hash(self):
        # Test __hash__ method
        agent_synergy1 = AgentSynergy("MainAgent", self.synergy1)
        agent_synergy2 = AgentSynergy("MainAgent", self.synergy1)
        agent_synergy3 = AgentSynergy("MainAgent", self.synergy2)

        self.assertEqual(hash(agent_synergy1), hash(agent_synergy2))
        self.assertNotEqual(hash(agent_synergy1), hash(agent_synergy3))

    def test_set(self):
        agent_synergy1 = AgentSynergy("MainAgent", self.synergy1)
        agent_synergy2 = AgentSynergy("MainAgent", self.synergy1)
        agent_synergy3 = AgentSynergy("MainAgent", self.synergy2)

        agent_synergies = set()
        agent_synergies.add(agent_synergy1)
        agent_synergies.add(agent_synergy2)
        agent_synergies.add(agent_synergy3)
        self.assertEqual(len(agent_synergies), 2)


class TestTaskStatistics(unittest.TestCase):
    def setUp(self):
        # Set up the test environment with a sample Statistics object
        self.statistics = Statistics(expected_duration=10.0, duration_std_dev=1.0)
        self.task_stats = TaskStatistics("Task1", "Agent1", self.statistics)

    def test_get_task_name(self):
        # Test the get_task_name method
        self.assertEqual(self.task_stats.get_task_name(), "Task1")

    def test_get_agent_name(self):
        # Test the get_agent_name method
        self.assertEqual(self.task_stats.get_agent_name(), "Agent1")

    def test_get_expected_duration(self):
        # Test the get_expected_duration method
        self.assertEqual(self.task_stats.get_expected_duration(), 10.0)

    def test_get_duration_std_dev(self):
        # Test the get_duration_std_dev method
        self.assertEqual(self.task_stats.get_duration_std_dev(), 1.0)

    def test_get_statistics(self):
        # Test the get_statistics method
        expected_agent_stats = AgentStats(agent_name="Agent1", statistics=self.statistics)
        self.assertEqual(self.task_stats.get_statistics(), expected_agent_stats)

    def test_eq(self):
        # Test the __eq__ method
        same_task_stats = TaskStatistics("Task1", "Agent1", self.statistics)
        different_task_stats = TaskStatistics("Task2", "Agent2", self.statistics)

        self.assertTrue(self.task_stats == same_task_stats)
        self.assertFalse(self.task_stats == different_task_stats)

    def test_hash(self):
        # Test the __hash__ method
        same_task_stats = TaskStatistics("Task1", "Agent1", self.statistics)
        different_task_stats = TaskStatistics("Task2", "Agent2", self.statistics)

        self.assertEqual(hash(self.task_stats), hash(same_task_stats))
        self.assertNotEqual(hash(self.task_stats), hash(different_task_stats))

    def test_set(self):
        # Test the length of a set after adding different TaskStatistics objects
        task_stats_set = set()
        task_stats_set.add(self.task_stats)  # Add one TaskStatistics object
        self.assertEqual(len(task_stats_set), 1)

        same_task_stats = TaskStatistics("Task1", "Agent1", self.statistics)
        task_stats_set.add(same_task_stats)
        self.assertEqual(len(task_stats_set), 1)  # Length should remain the same

        # Add a different TaskStatistics object
        different_task_stats = TaskStatistics("Task2", "Agent2", self.statistics)
        task_stats_set.add(different_task_stats)
        self.assertEqual(len(task_stats_set), 2)  # Length should increase by 1


class TestTaskSynergies(unittest.TestCase):
    def setUp(self):
        # Create a sample TaskSynergies object for testing
        self.task_synergies = TaskSynergies("MainTask", "MainAgent")

    def test_add_synergy(self):
        # Test adding a synergy
        self.task_synergies.add_synergy("ParallelTask", "ParallelAgent", 1.0, 0.5)
        self.assertTrue(
            self.task_synergies.has_synergy(Synergy("ParallelTask", "ParallelAgent", AtomicSynergy(1.0, 0.5))))

    def test_add_synergy_duplicate(self):
        # Test adding a duplicate synergy
        with patch('builtins.print') as mock_print:
            self.task_synergies.add_synergy("ParallelTask", "ParallelAgent", 1.0, 0.5)
            self.task_synergies.add_synergy("ParallelTask", "ParallelAgent", 2.0, 0.6)
            self.assertEqual(mock_print.call_count, 1)  # Ensure warning message is printed once

    def test_add_synergy_same_main_task_agent(self):
        # Test adding a synergy with the same main task and agent name
        with self.assertRaises(ValueError):
            self.task_synergies.add_synergy("MainTask", "MainAgent", 1.0, 0.5)

    def test_get_synergy(self):
        # Test getting a synergy
        self.task_synergies.add_synergy("ParallelTask", "ParallelAgent", 1.0, 0.5)
        self.assertEqual(self.task_synergies.get_synergy("ParallelTask", "ParallelAgent"),
                         Synergy("ParallelTask", "ParallelAgent", AtomicSynergy(1.0, 0.5)))

    def test_get_synergy_not_found(self):
        # Test getting a non-existing synergy
        self.assertIsNone(self.task_synergies.get_synergy("NonExistingTask", "NonExistingAgent"))

    def test_has_synergy(self):
        # Test checking if a synergy exists
        self.task_synergies.add_synergy("ParallelTask", "ParallelAgent", 1.0, 0.5)
        self.assertTrue(
            self.task_synergies.has_synergy(Synergy("ParallelTask", "ParallelAgent", AtomicSynergy(1.0, 0.5))))
        self.assertFalse(
            self.task_synergies.has_synergy(Synergy("NonExistingTask", "NonExistingAgent", AtomicSynergy(1.0, 0.5))))

    def test_get_synergies(self):
        # Test getting all synergies
        self.task_synergies.add_synergy("ParallelTask1", "ParallelAgent1", 1.0, 0.5)
        self.task_synergies.add_synergy("ParallelTask2", "ParallelAgent2", 2.0, 0.6)
        self.assertEqual(self.task_synergies.get_synergies(),
                         {Synergy("ParallelTask1", "ParallelAgent1", AtomicSynergy(1.0, 0.5)),
                          Synergy("ParallelTask2", "ParallelAgent2", AtomicSynergy(2.0, 0.6))})

    @patch('statistics_utils.AgentSynergy', MagicMock(side_effect=ValueError))
    def test_get_agent_synergies_error(self):
        # Test getting agent synergies with an error (the patch force the )
        self.task_synergies.add_synergy("ParallelTask1", "ParallelAgent1", 1.0, 0.5)
        with self.assertRaises(ValueError):
            self.task_synergies.get_agent_synergies()


if __name__ == '__main__':
    unittest.main()
