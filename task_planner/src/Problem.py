from Task import Task
from dataclasses import dataclass, field
from typing import List, Dict, Optional


@dataclass
class Problem:
    task_list: List[Task] = field(default_factory=list, init=False)

    def add_task(self, task):
        self.task_list.append(task)

    def get_task_list_as_dictionary(self) -> Dict[str, Task]:
        return {task.get_id(): task for task in self.task_list}

    def consistency_check(self):
        tasks_dict = self.get_task_list_as_dictionary()

        # Check precedence consistency (exists & not-loop)
        for task in self.task_list:
            # if not any(precedence_task in tasks_dict.keys() for precedence_task in task.get_precedence_constraints()):
            #     return False
            for precedence_task in task.get_precedence_constraints():
                # Check if exist all precedence constraints among task_ids
                if precedence_task not in tasks_dict.keys():
                    print(f"The precedence task: {precedence_task}, of task: {task.get_id()}, does not exist.")
                    return False

                # Check if loop constraints
                if task.get_id() in tasks_dict[precedence_task].get_precedence_constraints():
                    print(f"Loop constraints between: {task.get_id()}, and: {task.get_id()}.")
                    return False
        return True


