from dataclasses import dataclass, field
from typing import List, Dict, Optional


@dataclass
class Task:
    id: str
    type: str
    agents: List[str]
    precedence_constraints: List[str]
    exp_duration: float = field(default=None, init=False)
    synergy: Dict[str, float] = field(default=None, init=False)

    def update_duration(self, duration: float) -> bool:
        assert duration > 0
        if duration < 0:
            return False
        self.exp_duration = duration
        return True

    def update_synergy(self, synergy: List[float]) -> None:
        self.synergy = synergy

    def update_agents(self, agents: List[str]) -> None:
        self.agents = agents

    def add_agent(self, agent: List[str]) -> None:
        if agent not in self.agents:
            self.agents.append(agent)

    def update_precedence_constraints(self) -> None:
        pass

    def get_duration(self) -> float:
        return self.exp_duration

    def get_synergy(self, task: str) -> Optional[str]:
        if task in self.synergy:
            return self.synergy[task]
        return None

    def get_id(self) -> str:
        return self.id

    def get_agents(self) -> List[str]:
        return self.agents

    def get_precedence_constraints(self) -> Optional[str]:
        return self.precedence_constraints

    def check_precedence_constraint(self, task: str) -> bool:
        if task in self.precedence_constraints:
            return True
        return False
