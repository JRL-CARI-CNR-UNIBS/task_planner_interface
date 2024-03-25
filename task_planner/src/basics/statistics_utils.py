from dataclasses import dataclass, field
from typing import Set, Optional
from utils import UserMessages


@dataclass
class Statistics:
    expected_duration: float
    duration_std_dev: float

    def __post_init__(self):
        if self.expected_duration < 0 or self.duration_std_dev < 0:
            raise ValueError("Statistics cannot have expected duration or std. dev less than 0")

    def get_expected_duration(self) -> float:
        return self.expected_duration

    def get_duration_std_dev(self) -> float:
        return self.duration_std_dev


@dataclass
class AgentStats:
    agent_name: str
    statistics: Statistics

    def get_statistics(self):
        return self.statistics

    def get_agent_name(self):
        return self.agent_name

    def __hash__(self):
        return hash(self.agent_name)

    def __eq__(self, other):
        return isinstance(other, AgentStats) and self.agent_name == other.agent_name


def get_max_duration(agents_statistics: Set[AgentStats]) -> float:
    if agents_statistics:
        return max({agent_stat.get_statistics().get_expected_duration() for agent_stat in agents_statistics})
    # TODO: Reasonin in what to do.
    raise ValueError("Agent statistics must ...")


@dataclass
class AtomicSynergy:
    expected_synergy: float
    synergy_std_dev: Optional[float]

    def __post_init__(self):
        if self.expected_synergy < 0:
            raise ValueError(UserMessages.SYNERGY_MUST_POSITIVE.value)
        if self.synergy_std_dev:
            if self.synergy_std_dev < 0:
                raise ValueError(UserMessages.SYNERGY_MUST_POSITIVE.value)


@dataclass
class Synergy:
    parallel_task_name: str
    parallel_agent_name: str
    synergy: AtomicSynergy

    def get_parallel_task_name(self):
        return self.parallel_task_name

    def get_parallel_agent_name(self):
        return self.parallel_agent_name

    def get_synergy(self):
        return self.synergy

    def __hash__(self):
        return hash((self.parallel_task_name, self.parallel_agent_name))

    def __eq__(self, other):
        return (isinstance(other, Synergy) and
                self.parallel_task_name == other.parallel_task_name and
                self.parallel_agent_name == other.parallel_agent_name)

    # synergy_value: float
    # std_dev: float

    # def __post__init__(self):
    #     if self.synergy_value < 0 or self.std_dev < 0:
    #         raise ValueError(UserMessages.SYNERGY_MUST_POSITIVE.value)
    # TODO: Check this: where is it used?
    # and
    #         self.synergy_value == other.synergy_value and
    #         self.std_dev == other.std_dev)


@dataclass
class AgentSynergy:
    main_agent_name: str
    synergy: Synergy

    def __post_init__(self):
        if self.synergy.parallel_agent_name == self.main_agent_name:
            raise ValueError(
                f"Other agent name: ({self.synergy.parallel_agent_name}) must differ by main one: "
                f"({self.main_agent_name})")

    def get_main_agent(self):
        return self.main_agent_name

    def get_synergy(self):
        return self.synergy

    def get_parallel_agent_name(self):
        return self.synergy.get_parallel_agent_name()

    def __hash__(self):
        return hash((self.main_agent_name, self.synergy.parallel_agent_name, self.synergy.parallel_task_name))

    def __eq__(self, other):
        return (isinstance(other, AgentSynergy) and
                self.main_agent_name == other.main_agent_name and
                self.synergy == other.synergy)


@dataclass
class TaskStatistics:
    task_name: str
    agent_name: str
    statistics: Statistics

    def get_expected_duration(self) -> float:
        return self.statistics.get_expected_duration()

    def get_duration_std_dev(self) -> float:
        return self.statistics.get_duration_std_dev()

    def get_statistics(self) -> Statistics:
        return AgentStats(agent_name=self.agent_name,
                          statistics=self.statistics)

    def get_task_name(self):
        return self.task_name

    def get_agent_name(self):
        return self.agent_name

    def __hash__(self):
        return hash((self.task_name, self.agent_name))

    def __eq__(self, other):
        return (isinstance(other,
                           TaskStatistics) and
                other.task_name == self.task_name and
                other.agent_name == self.agent_name)

    # def __eq__(self, other):
    #     return (isinstance(other, TaskStatistics) and
    #             self.task_name == other.task_name and
    #             self.agent_name == other.agent_name)


# def __eq__(self, other):
#     return isinstance(other,str) and other


@dataclass
class TaskSynergies:
    main_task_name: str
    main_agent_name: str
    synergies: Set[Synergy] = field(default_factory=set, init=False)

    def get_main_task_name(self):
        return self.main_task_name

    def get_main_agent_name(self):
        return self.main_agent_name

    def add_synergy(self,
                    parallel_task_name: str,
                    parallel_agent_name: str,
                    synergy_value: float,
                    std_dev: Optional[float] = None):
        if parallel_task_name == self.main_task_name and parallel_agent_name == self.main_agent_name:
            raise ValueError(
                f"Other task and agent name: ({parallel_task_name}, {parallel_agent_name}) must differ by main one: "
                f"({self.main_task_name}, {self.main_agent_name}). Not added")
        try:
            synergy = Synergy(
                parallel_task_name=parallel_task_name,
                parallel_agent_name=parallel_agent_name,
                synergy=AtomicSynergy(synergy_value,
                                      std_dev)
                # synergy_value=synergy_value,
                # std_dev=std_dev
            )
        except ValueError as exc:
            raise exc
        if synergy in self.synergies:
            print(f"Warning, synergy between task: {self.main_task_name} agent: {self.main_agent_name}"
                  f"and task: {parallel_task_name} agent: {parallel_agent_name} duplicated, old removed")
        self.synergies.add(synergy)

    def get_synergy(self, parallel_task_name: str, parallel_agent_name: str) -> Optional[Synergy]:
        for synergy in self.synergies:
            if synergy.parallel_task_name == parallel_task_name and synergy.parallel_agent_name == parallel_agent_name:
                return synergy
        return None

    # def get_agent_synergies(self, parallel_agent_name: str) -> set:
    #     return {synergy for synergy in self.synergies if synergy.parallel_agent_name == parallel_agent_name}

    def get_synergies(self) -> Set[Synergy]:
        return self.synergies

    # def get_all_synergies(self) -> set:
    #     return self.synergies

    def has_synergy(self, synergy: Synergy) -> bool:
        return synergy in self.synergies

    def get_agent_synergies(self) -> Set[AgentSynergy]:
        agent_synergies: Set[AgentSynergy] = set()
        for synergy in self.synergies:
            try:
                agent_synergy = AgentSynergy(main_agent_name=self.main_agent_name,
                                             synergy=synergy)
                agent_synergies.add(agent_synergy)
            except ValueError as exc:
                raise exc
        return agent_synergies
