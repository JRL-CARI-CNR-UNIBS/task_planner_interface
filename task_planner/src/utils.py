from enum import Enum

from Task import TaskSolution
from typing import List
import pandas as pd
from datetime import datetime
import plotly.express as px
import plotly.express as px


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

    ### Database Messages ###
    CONNECTION_OK = Color.GREEN.value + "Connection to db executed" + Color.END.value
    DATABASE_DISCONNECTED = Color.RED.value + "Disconnected from database" + Color.END.value
    CONNECTION_FAILED = Color.RED.value + "Connection to db failed: Request Timeout" + Color.END.value
    CONNECTION_LOST = Color.RED.value + "Connection to Database lost" + Color.END.value
    UPDATE_OK = Color.GREEN.value + "Update performed correctly" + Color.END.value

    ### Task planning info ###
    TASK_DUPLICATION = Color.RED.value + "Task: {}, is duplicated" + Color.RED.value
    CONSISTENCY_CHECK_FAILED = Color.RED.value + "Consistency Check of the problem failed" + Color.RED.value
    PROBLEM_NOT_FEASIBLE = Color.RED.value + "The problem is infeasible: check the constraints!" + Color.RED.value
    PROBLEM_NOT_FEASIBLE_DATA = Color.RED.value + "The problem is infeasible (data reasoning)!" + Color.RED.value


def show_timeline(problem_solution: List[TaskSolution]) -> None:
    solution = []
    for task in problem_solution:
        solution.append(dict(Task=task.get_task().get_id(),
                             Start=datetime.fromtimestamp(task.get_start_time()).strftime("2020-04-06 %I:%M:%S"),
                             Finish=datetime.fromtimestamp(task.get_end_time()).strftime("2020-04-06 %I:%M:%S"),
                             Agents=task.get_assignment()))
    df = pd.DataFrame(solution)
    fig = px.timeline(df, x_start="Start", x_end="Finish", y="Agents", color="Task", title="TimeLine")
    fig.update_layout(xaxis_title="Time")
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


class Behaviour(Enum):
    DISCRETE = 1
    CONTINUOUS = 2


class Overlapping(Enum):
    INSIDE = 1
    OUTSIDE = 2


class Objective(Enum):
    MAKESPAN = 1
    SUM_T_END = 2
    SUM_T_START_END = 3
    SYNERGY = 4
    SUM_T_END = 5
