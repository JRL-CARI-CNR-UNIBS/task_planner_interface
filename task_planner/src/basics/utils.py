from dataclasses import dataclass
from enum import Enum


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
    ORANGE = '\033[33m'
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
    WAITING_SERVICE = "Waiting Service: {}"
    SERVICE_READY = "Service Ready: {}"

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
    CUSTOM_ORANGE = Color.ORANGE.value + "{}" + Color.END.value
    CUSTOM_YELLOW = Color.YELLOW.value + "{}" + Color.END.value
    CUSTOM_GREEN = Color.GREEN.value + "{}" + Color.END.value
    CUSTOM_CYAN = Color.CYAN.value + "{}" + Color.END.value
    CUSTOM_DARKCYAN = Color.DARKCYAN.value + "{}" + Color.END.value

    RECIPE_NAME_FORMAT = "{}_{}_{}"
    UNABLE_GO_ON = " ---------------- Unable to go on! -------------------- "
    UNABLE_CAPABILITIES = "Unable to get agents capabilities (task properties)"
    UNABLE_STATS = "Unable to get tasks statistics"

    ### Knowledge Base
    TASK_NOT_PRESENT = Color.RED.value + "Task {} not present in Knowledge Base" + Color.END.value
    SYNERGY_MUST_POSITIVE = Color.RED.value + "Synergy must have positive value and std. dev." + Color.END.value


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

    def __hash__(self):
        return hash(self.agent_name)


@dataclass
class Synergy:
    other_task_name: str
    other_agent_name: str
    synergy_value: float
    std_dev: float

    def __post_init__(self):
        if self.synergy_value < 0 or self.std_dev < 0:
            raise ValueError(UserMessages.SYNERGY_MUST_POSITIVE.value)

    def __hash__(self):
        return hash((self.other_task_name, self.other_agent_name))


@dataclass
class AgentSynergy:
    main_agent_name: str
    synergy: Synergy

    def __post_init__(self):
        if self.synergy.other_task_name == self.main_agent_name:
            raise ValueError(
                f"Other agent name: ({self.synergy.other_agent_name}) must differ by main one: "
                f"({self.main_agent_name})")

    def __hash__(self):
        return hash((self.synergy.other_task_name, self.other_agent_name))


class DataLoadingError(Exception):
    pass
