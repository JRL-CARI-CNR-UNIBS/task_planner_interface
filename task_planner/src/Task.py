from dataclasses import dataclass, field
from typing import List, Dict, Optional, overload


@dataclass
class Task:
    id: str
    type: str
    agents: List[str]
    precedence_constraints: List[str]
    exp_duration: Dict[str, float] = field(default=None, init=False)
    synergy: Dict[str, Dict[str, float]] = field(default=None, init=False)

    def update_duration(self, agent: str, duration: float) -> bool:
        assert agent in self.agents
        assert duration > 0

        if self.exp_duration is None:
            self.exp_duration = {}
        if duration < 0:
            return False
        if agent not in self.agents:
            return False

        self.exp_duration[agent] = duration
        return True

    def update_synergy(self, synergy: Dict[str, float]) -> None:
        self.synergy = synergy

    def update_agents(self, agents: List[str]) -> None:
        self.agents = agents

    def add_agent(self, agent: str) -> None:
        if agent not in self.agents:
            self.agents.append(agent)

    def update_precedence_constraints(self) -> None:
        pass

    @overload
    def get_duration(self) -> Dict[str, float]:
        ...

    @overload
    def get_duration(self, agent: str) -> float:
        ...

    def get_duration(self, *args):
        if len(args) == 1 and isinstance(args[0], str):
            agent = args[0]
            assert agent in self.agents
            if agent not in self.agents:
                raise Exception
            return self.exp_duration[agent]
        elif len(args) == 0:
            return self.exp_duration
        else:
            raise NotImplemented

    def get_synergy(self, task: str) -> Optional[str]:
        if task in self.synergy:
            return self.synergy[task]
        return None

    def get_id(self) -> str:
        return self.id

    def get_type(self) -> str:
        return self.type

    def get_agents(self) -> List[str]:
        return self.agents

    def get_precedence_constraints(self) -> Optional[str]:
        return self.precedence_constraints

    def check_precedence_constraint(self, task: str) -> bool:
        if task in self.precedence_constraints:
            return True
        return False
