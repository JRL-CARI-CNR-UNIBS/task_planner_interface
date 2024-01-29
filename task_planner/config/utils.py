import os
from enum import Enum

from Task import Task, TaskSolution
# from Problem import Problem

from typing import List, Optional
import pandas as pd
from datetime import datetime
import plotly.express as px
from pathlib import Path

import ruamel.yaml


class Color(Enum):
    """
        Utility Enum class with color constant definition
    """

    PURPLE = '\033[95m'
    CYAN = '\033[96m'
    DARKCYAN = '\033[36m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'


class UserMessages(Enum):
    """
        Utility Enum class with color constant definition
    """

    ### Ros Messages ###
    PARAM_NOT_DEFINED_ERROR = Color.RED.value + "Parameter: {} not defined" + Color.END.value
    PARAM_NOT_WELL_DEFINED = Color.RED.value + "Parameter: {} not well defined" + Color.END.value

    SERVICE_FAILED = Color.RED.value + "Service call failed : {}" + Color.END.value
    SERVICE_CALLBACK = Color.GREEN.value + "Service call {} received" + Color.END.value

    ### Generic Messages ###
    INPUT_MESSAGE_WAIT = "Press Enter to continue..."
    SUCCESSFUL = Color.GREEN.value + "Successfully executed" + Color.END.value
    NOT_SUCCESSFUL = "Not Successfully executed"
    READY = Color.GREEN.value + "Ready" + Color.END.value
    CHECK_OK = Color.GREEN.value + "Check performed correctly" + Color.END.value
    IMPOSSIBLE_TO_GO_ON = Color.RED.value + "--- Impossible to continue ---" + Color.END.value

    ### Database Messages ###
    CONNECTION_OK = Color.GREEN.value + "Connection to db executed" + Color.END.value
    DATABASE_DISCONNECTED = Color.RED.value + "Disconnected from database" + Color.END.value
    CONNECTION_FAILED = Color.RED.value + "Connection to db failed: Request Timeout" + Color.END.value
    CONNECTION_LOST = Color.RED.value + "Connection to Database lost" + Color.END.value
    UPDATE_OK = Color.GREEN.value + "Update performed correctly" + Color.END.value

    ### Task planning info ###
    TASK_DUPLICATION = Color.RED.value + "Task: {}, is duplicated" + Color.END.value
    CONSISTENCY_CHECK_FAILED = Color.RED.value + "Consistency Check of the problem failed" + Color.END.value
    PROBLEM_NOT_FEASIBLE = Color.RED.value + "The problem is infeasible: check the constraints!" + Color.END.value
    PROBLEM_NOT_FEASIBLE_DATA = Color.RED.value + "The problem is infeasible (data reasoning)!" + Color.END.value
    PLAN_FAILED = Color.RED.value + "The plan is failed" + Color.END.value
    # DISPATCH_TASK_MSG = Color.CYAN.value + "Published Task: {}, Agent: {}, T_start: {} T_end: {}" + Color.END.value

    ### Task "Dispatcher" (main)
    DISPATCH_TASK_MSG = Color.CYAN.value + "Published Task: {}, Agent: {}" + Color.END.value
    FEEDBACK_TASK_MSG = Color.DARKCYAN.value + "Received feedback: {}, Agent: {}" + Color.END.value
    FEEDBACK_FAILED_TASK_MSG = Color.RED.value + "Received feedback: {}, Agent: {}, outcome: Failed" + Color.END.value

    FEEDBACK_TASK_EMPTY_LIST_MSG = Color.YELLOW.value + "Receive a feedback of empty tasks" + Color.END.value
    FEEDBACK_WITHOUT_DISPATCH = Color.YELLOW.value + "Receive a feedback without have dispatched any task" + Color.END.value
    TIMEOUT = Color.RED.value + "------- Timeout, waiting feedback of: {}  -------" + Color.END.value

    PLAN_FINISHED = Color.GREEN.value + "----------- The plan is finished --------------" + Color.GREEN.value

    ### Custom colored messages
    CUSTOM_RED = Color.RED.value + "{}" + Color.END.value
    CUSTOM_YELLOW = Color.YELLOW.value + "{}" + Color.END.value
    CUSTOM_GREEN = Color.GREEN.value + "{}" + Color.END.value
    CUSTOM_CYAN = Color.CYAN.value + "{}" + Color.END.value
    CUSTOM_DARKCYAN = Color.DARKCYAN.value + "{}" + Color.END.value

    RECIPE_NAME_FORMAT = "{}_{}_{}"
    UNABLE_GO_ON = " ---------------- Unable to go on! -------------------- "
    UNABLE_CAPABILITIES = "Unable to get agents capabilities (task properties)"
    UNABLE_STATS = "Unable to get tasks statistics"


def show_timeline(problem_solution: List[TaskSolution],
                  solution_name: Optional[str] = "Timeline") -> None:
    solution = []
    for task in problem_solution:
        solution.append(dict(Task=task.get_task().get_type(),
                             Start=datetime.fromtimestamp(task.get_start_time()).strftime("2020-04-06 %I:%M:%S"),
                             Finish=datetime.fromtimestamp(task.get_end_time()).strftime("2020-04-06 %I:%M:%S"),
                             Agents=task.get_assignment()))
    df = pd.DataFrame(solution)
    print(df)
    fig = px.timeline(df, x_start="Start", x_end="Finish", y="Agents", color="Task", title=solution_name)
    fig.update_layout(xaxis_title="Time")
    fig.show()


def show_timeline_irim(problem_solution: List[TaskSolution],
                       solution_name: Optional[str] = "Timeline") -> None:
    print("ciao")
    color_mapping = {
        "pick_orange_box": "coral",
        "place_orange_box": "peachpuff",  # Arancio più tenue
        "pick_blue_box": "deepskyblue",  # Blu più tenue
        "place_blue_box_ur5_on_guide": "lightsteelblue",  # Blu chiaro
        "place_blue_box_human_right_arm": "lightsteelblue",  # Blu chiaro
        "pick_white_box": "lightgray",  # Grigio chiaro
        "place_white_box": "whitesmoke",  # Bianco fumé
    }
    color_mapping = {
        "pick_orange_box": "#ffc048",
        "place_orange_box": "peachpuff",  # Arancio più tenue
        "pick_blue_box": "#4a69bd",  # Blu più tenue
        "place_blue_box": "#6a89cc",  # Blu più tenue
        "place_blue_box_ur5_on_guide": "#6a89cc",  # Blu chiaro
        "place_blue_box_human_right_arm": "#6a89cc",  # Blu chiaro
        "pick_white_box": "#95a5a6",  # Grigio chiaro
        "place_white_box": "#bdc3c7",  # Bianco fumé,
        "place_blue_box_ur5_on_guide": "#6a89cc"
    }
    name_mapping = {
        "pick_orange_box": "Pick Orange Box",
        "place_orange_box": "Place Orange Box",  # Arancio più tenue
        "pick_blue_box": "Pick Blue Box",  # Blu più tenue
        "place_blue_box": "Place Blue Box",  # Blu più tenue
        "place_blue_box_ur5_on_guide": "Place Blue Box",  # Blu chiaro
        "place_blue_box_human_right_arm": "Place Blue Box",  # Blu chiaro
        "pick_white_box": "Pick White Box",  # Grigio chiaro
        "place_white_box": "Place White Box",  # Bianco fumé,
        "place_blue_box_ur5_on_guide": "Place Blue Box"
    }
    color_mapping = {
        "Pick Orange Box": "#ffc048",
        "Place Orange Box": "peachpuff",  # Arancio più tenue
        "Pick Blue Box": "#4a69bd",  # Blu più tenue
        "Place Blue Box": "#6a89cc",  # Blu più tenue
        "place_blue_box_ur5_on_guide": "#6a89cc",  # Blu chiaro
        "place_blue_box_human_right_arm": "#6a89cc",  # Blu chiaro
        "Pick White Box": "#95a5a6",  # Grigio chiaro
        "Place White Box": "#bdc3c7",  # Bianco fumé,
        "place_blue_box_ur5_on_guide": "#6a89cc"
    }
    agent_order = ["Robot", "Human"]  # Sostituisci con gli agenti effettivi
    # agent_order = ["ur5_on_guide", "human_right_arm"]  # Sostituisci con gli agenti effettivi
    AGENT_MAPPING = {"ur5_on_guide": "Robot",
                     "human_right_arm": "Human"}
    solution = []
    for task in problem_solution:

        print(task.get_task().get_type())
        agent = AGENT_MAPPING.get(task.get_assignment(), task.get_assignment())

        # solution.append(dict(Task=task.get_task().get_id()[:-2],
        # solution.append(dict(Task=task.get_task().get_type(),
        solution.append(dict(Task=name_mapping.get(task.get_task().get_type(),task.get_task().get_type()),
                             Start=datetime.fromtimestamp(task.get_start_time()).strftime("2020-04-06 %I:%M:%S"),
                             Finish=datetime.fromtimestamp(task.get_end_time()).strftime("2020-04-06 %I:%M:%S"),
                             Agents=agent,
                             AgentOrder=agent_order.index(agent),  # Ordine degli agenti
                             TaskColor=color_mapping.get(task.get_task().get_type(), "gray")))
    df = pd.DataFrame(solution)
    df = df.sort_values(by="AgentOrder")

    print(df)
    # fig = px.timeline(df, x_start="Start", x_end="Finish", y="Agents", color="Task", title=solution_name)
    task_color_map = {task: color_mapping.get(task, "gray") for task in df["Task"].unique()}

    fig = px.timeline(df, x_start="Start", x_end="Finish", y="Agents", color="Task", color_discrete_map=task_color_map,
                      title=solution_name)
    fig.update_layout(xaxis_title="Time", yaxis_title="Agents")

    # fig.update_layout(xaxis_title="Time")
    fig.show()


def show_gantt(problem_solution: List[TaskSolution]) -> None:
    solution = []
    for task in problem_solution:
        solution.append(dict(Task=task.get_task().get_type(),
                             Start=datetime.fromtimestamp(task.get_start_time()).strftime("2020-04-06 %I:%M:%S"),
                             Finish=datetime.fromtimestamp(task.get_end_time()).strftime("2020-04-06 %I:%M:%S"),
                             Agents=task.get_assignment()))
    df = pd.DataFrame(solution)
    fig = px.timeline(df, x_start="Start", x_end="Finish", y="Task", color="Agents", title="Gantt")
    fig.update_layout(xaxis_title="Time")
    fig.show()
    fig = px.timeline(df, x_start="Start", x_end="Finish", y="Task", color="Task", title="Gantt")
    fig.update_layout(xaxis_title="Time")
    fig.show()


def save_planning_solution_to_yaml(solution: List[TaskSolution], path: Path, problem):
    solution_to_save = dict()
    for task_sol in solution:
        task_sol: TaskSolution
        solution_to_save[task_sol.get_task().get_id()] = {"t_start": task_sol.get_start_time(),
                                                          "t_end": task_sol.get_end_time(),
                                                          "agent": task_sol.get_assignment()}
    yaml = ruamel.yaml.YAML(typ='rt')
    yaml.preserve_quotes = True
    with open(path, 'wb') as f:
        yaml.dump(solution_to_save, f)


# def load_solution_from_yaml(path: Path, problem_to_solve: Problem):
#     yaml = ruamel.yaml.YAML(typ='rt')
#     yaml.preserve_quotes = True
#     solution = list()
#     problem = problem_to_solve.get_task_list_as_dictionary()
#     for task in solution:
#         if task not in problem:
#             raise Exception
#
#         solution.append(TaskSolution(problem[task],
#                                      solution[task]["t_start"],
#                                      solution[task]["t_emd"],
#                                      solution[task]["agent"]))
#     return solution

def show_solution_from_yaml(path: Path,
                            solution_name: Optional[str] = "Timeline"):
    solution = load_solution_from_yaml(path, solution_name)
    show_timeline_irim(solution, solution_name)


def load_solution_from_yaml(path: Path,
                            solution_name: Optional[str] = "Timeline") -> List[TaskSolution]:
    import yaml
    if (not path.exists()) or (not path.is_file()):
        raise ValueError("THe provided file is not a valid, not a file")

    loaded_sol = dict()
    with open(path, "r") as stream:
        try:
            loaded_sol = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            raise yaml.YAMLError("Error Loading yaml")

    task_feature = ["t_start", "t_end", "agent"]
    solution = list()
    for task_name, task_solution in loaded_sol.items():
        # TODO: Sistemare name in type
        task_name = task_name[:-2]
        if all(feature in task_solution.keys() for feature in task_feature):
            t_start = task_solution["t_start"]
            t_end = task_solution["t_end"]
            agent = task_solution["agent"]
            task = Task(id=task_name,
                        type=task_name,
                        agents_constraint="",
                        precedence_constraints="",
                        soft_precedence_constraints=""
                        )
            task_sol = TaskSolution(task=task,
                                    t_start=t_start,
                                    t_end=t_end,
                                    assignment=agent)
            solution.append(task_sol)
    return solution
    # show_timeline(solution,
    #               solution_name)

    # problem = problem_to_solve.get_task_list_as_dictionary()
    # for task in solution:
    #     if task not in problem:
    #         raise Exception
    #
    #     solution.append(TaskSolution(problem[task],
    #                                  solution[task]["t_start"],
    #                                  solution[task]["t_emd"],
    #                                  solution[task]["agent"]))
    # return solution


def show_solutions_from_folder(folder_path: Path):
    if not isinstance(folder_path, Path) and isinstance(folder_path, str):
        folder_path = Path(folder_path)
    if (not folder_path.exists()) or (not folder_path.is_dir()):
        raise ValueError("The provided file is not a valid, not a file")
    from os import listdir
    for file_name in listdir(folder_path):
        file_path = folder_path.joinpath(file_name)
        if file_path.is_file():
            print(file_path)
            show_solution_from_yaml(file_path, file_name)
        else:
            print(f"Provided folder contains also: {file_path} that is not a file")


class Behaviour(Enum):
    DISCRETE = 1
    CONTINUOUS = 2


class Overlapping(Enum):
    INSIDE = 1
    OUTSIDE = 2


class Objective(Enum):
    MAKESPAN = 1
    SUM_T_START = 2
    SUM_T_START_END = 3
    SYNERGY = 4
    SUM_T_END = 5
    ACTUAL_MAKESPAN = 6
    OTHER = 7