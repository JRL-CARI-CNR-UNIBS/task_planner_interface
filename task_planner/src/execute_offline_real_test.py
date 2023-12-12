#!/usr/bin/env python3

import rospy
from utils import *
from Task import Task
from Problem import Problem
from TaskPlanner import TaskPlanner
from TaskPlannerHumanAware import TaskPlannerHumanAware
from TaskPlannerHumanAwareRelaxed import TaskPlannerHumanAwareRelaxed

from TaskPlannerHumanAwareEasier import TaskPlannerHumanAwareEasier
from TaskPlannerHumanAwareLast import TaskPlannerHumanAwareLast
from TaskPlannerTest import TaskPlannerTest

from TaskPlannerSynergistic import TaskPlannerSynergistic
from TaskPlannerSynergisticEasier import TaskPlannerSynergisticEasier
from TaskPlannerSynergisticBand import TaskPlannerSynergisticBand
from TaskPlannerAreas import TaskPlannerAreas
from task_planner_interface_msgs.srv import DeleteRecipe, DeleteRecipeRequest, DeleteRecipeResponse

from Prove import Prove
from TaskDispatcher import TaskDispatcher
from std_srvs.srv import Trigger
from std_msgs.msg import String
import copy
import numpy as np
from pathlib import Path
import yaml
from pathlib import Path
from os import listdir
from os.path import isfile, join

RECIPE_NAME = {"base": "BASIC_SOLVER",
               "human_aware_easier": "RELAXED_HA_SOLVER",
               "areas": "NOT_NEIGHBORING_TASKS",
               "human_aware_complete": "COMPLETE_SOLVER"}
DATE_FORMAT = "%Y_%m_%d_%H:%M"


def params_exist(parameters: List[str]) -> bool:
    """
    Check if the parameters passed as argument exist
    Args:
        parameters: List of all parameters to check

    Returns: True if all parameters exist and False otherwise.

    """
    params_exist_check = True
    for param_name in parameters:
        if not rospy.has_param(param_name):
            rospy.logerr(UserMessages.PARAM_NOT_DEFINED_ERROR.value.format(param_name))
            params_exist_check = False
    return params_exist_check


def add_go_home(task_solution: List[TaskSolution], agents: List[str]):
    """
    Utility function that add the go_home task if two task are too far away from each other.
    Args:
        task_solution:List of TaskSolution
        agents: List of agentrs

    Returns:

    """
    task_solution: TaskSolution
    # 3 per il human aware
    # 1.4 per il caso base e aree
    # go_home_duration = 3 #era 1.4 qui e 1.5 sotto
    # go_home_duration = 1 # per il caso h-a easier??? bohh
    go_home_duration = 15

    go_home_duration_expanded = go_home_duration + 0.1
    task_solution.sort(key=lambda task_sol: task_sol.get_start_time())
    task_solutions = {agent: list(filter(lambda task_sol: task_sol.get_assignment()
                                                          == agent, task_solution)) for agent in
                      agents}
    for agent in agents:
        k = 0
        for id, agent_task in enumerate(task_solutions[agent][:-1]):
            if task_solutions[agent][id + 1].get_start_time() > agent_task.get_end_time() + go_home_duration_expanded:
                if k < 4:
                    task_solution.append(
                        TaskSolution(Task("go_home", "go_home", [agent], [], []),
                                     agent_task.get_end_time() + 0.1, agent_task.get_end_time() + go_home_duration,
                                     agent))
                k += 1

        if agent in task_solutions:
            if task_solutions[agent]:
                task_solution.append(
                    TaskSolution(Task("go_home", "go_home", [agent], [], []),
                                 task_solutions[agent][-1].get_end_time() + 0.1,
                                 task_solutions[agent][-1].get_end_time() + go_home_duration,
                                 agent))

    return task_solution


def compute_synergy_val(solution: TaskSolution) -> float:
    """
    Utility function that compute the total synergy index of a solution
    Args:
        solution: TaskSolution

    Returns: the synergy index of the plan solution

    """
    solution.sort(key=lambda task_sol: task_sol.get_start_time())

    problem_solution_agent = {agent: list(filter(lambda task_solution: task_solution.get_assignment()
                                                                       == agent, solution)) for agent in
                              ["ur5_on_guide", "human_right_arm"]}

    synergy_tot = 0.0
    for main_agent_task_sol in problem_solution_agent["ur5_on_guide"]:
        main_agent_task_sol: TaskSolution

        for parallel_agent_task_sol in problem_solution_agent["human_right_arm"]:
            parallel_agent_task_sol: TaskSolution
            overlapping = min(parallel_agent_task_sol.get_end_time(), main_agent_task_sol.get_end_time()) - max(
                parallel_agent_task_sol.get_start_time(), main_agent_task_sol.get_start_time())
            if overlapping <= 0:
                overlapping = 0
            synergy = main_agent_task_sol.get_task().get_synergy("ur5_on_guide", "human_right_arm",
                                                                 parallel_agent_task_sol.get_task().get_type())
            if not synergy:
                synergy = 1
            synergy_index = overlapping * (synergy - 1)
            synergy_tot += synergy_index

    # print(f"Synergy tot: {synergy_tot}")
    return synergy_tot


def get_optimization_name(recipe_name: str) -> str:
    for optimization_type in RECIPE_NAME.keys():
        if optimization_type in recipe_name:
            return optimization_type
    return "NOT_DEFINED_NAME"


def main():
    rospy.init_node("task_planner")
    parameters = ["~goal", "/agents", "/agents_group_names", "~repetitions", "~method"]

    if not params_exist(parameters):
        return 0

    task_goal = rospy.get_param("~goal")
    agents_name = rospy.get_param("/agents")
    agents_group_name_param = rospy.get_param("/agents_group_names")

    n_repetitions = rospy.get_param("~repetitions")
    if n_repetitions < 1:
        return 0

    method = rospy.get_param("~method")

    # Services and Publishers
    reload_scene_srv_client = rospy.ServiceProxy('/reload_scene', Trigger)
    pub_recipe_name = rospy.Publisher("set_recipe_name", String, queue_size=10)

    acquire_distance = rospy.get_param("acquire_distance", False)
    if acquire_distance:
        stop_distance_acq_srv_client = rospy.ServiceProxy("stop_distance_acq", Trigger)
        reset_distance_acq_srv_client = rospy.ServiceProxy("reset_distance_acq", Trigger)

    # Agents group name ("Agent name")
    agents_group_name = []
    for agent_type in agents_name.values():
        for agent in agent_type:

            if agent not in list(agents_group_name_param.keys()):
                rospy.logerr(UserMessages.CUSTOM_RED.value.format(
                    "Param: agents_group_names, does not contain group name of agent: " + str(agent)))
                return 0
            agents_group_name.append(agents_group_name_param[agent])

    print(agents_group_name)

    # Build the problem to solve
    problem_to_solve = Problem(agents_group_name)

    # Iterate the goal in yaml
    for task in task_goal:
        assert len(task.keys()) == 1
        if len(task.keys()) != 1:
            rospy.loginfo(UserMessages.PARAM_NOT_WELL_DEFINED.value.format("goal"))
            return 0

        task_id = list(task.keys())[0]
        task_properties = task[task_id]
        if not any(key in task_properties.keys() for key in ["task_name",
                                                             "required_agents",
                                                             "precedence_constraints",
                                                             "soft_precedence_constraints"]):
            rospy.logerr(UserMessages.PARAM_NOT_WELL_DEFINED.value.format("goal"))
            return 0

        task_obj = Task(task_id,
                        task_properties['task_name'],
                        task_properties["required_agents"],
                        task_properties["precedence_constraints"],
                        task_properties["soft_precedence_constraints"])

        try:
            problem_to_solve.add_task(task_obj)
        except Exception:  # TODO: Specify Excpetion
            rospy.logerr(UserMessages.TASK_DUPLICATION.value.format(task_id))
            return 0
    if not problem_to_solve.fill_task_agents():  # TODO : Manage as exception?
        rospy.loginfo(UserMessages.UNABLE_CAPABILITIES.value)
        rospy.logerr(UserMessages.UNABLE_GO_ON.value)
        return 0
    if not problem_to_solve.update_tasks_statistics():
        rospy.loginfo(UserMessages.UNABLE_STATS.value)
        rospy.logerr(UserMessages.UNABLE_GO_ON.value)
        return 0
    if not problem_to_solve.update_tasks_synergy():
        return 0

    rospy.loginfo(f"Consistency Check: {problem_to_solve.consistency_check()}")

    base_path = "/home/galois/projects/cells_ws/src/hrc_case_study_cell/hrc_case_study/hrc_case_study_task_planning/solutions/remaining/"

    if method == "A":
        path = Path(f"{base_path}base/todo_new")
        n_repetitions = 1
    elif method == "B":
        path = Path(f"{base_path}areas/todo_new/prove")
        n_repetitions = 1
    elif method == "C":
        path = Path(f"{base_path}relaxed")
        n_repetitions = 5
    elif method == "D":
        path = Path(f"{base_path}hatp")
        n_repetitions = 1
    else:
        return 0

    if not path.exists():
        raise ValueError("THe provided file is not a valid, not a file")
    solutions_files = os.listdir(path)

    only_interested_file = solutions_files
    only_interested_file = sorted(only_interested_file)

    recipe = 0
    data = "2023_10_26_19:29" # Data Mladen
    # data = datetime.today().strftime(DATE_FORMAT)

    for solution_name in only_interested_file:
        for repetition in range(0, n_repetitions):

            actual_problem_to_solve = copy.deepcopy(problem_to_solve)
            file_path = Path(join(path, solution_name))
            print(solution_name)

            loaded_sol = dict()
            with open(file_path, "r") as stream:
                try:
                    loaded_sol = yaml.safe_load(stream)
                except yaml.YAMLError as exc:
                    raise yaml.YAMLError("Error Loading yaml")

            task_feature = ["t_start", "t_end", "agent"]
            solution = list()
            for task_id, task_solution in loaded_sol.items():
                if actual_problem_to_solve.task_in_task_list(task_id):
                    if all(feature in task_solution.keys() for feature in task_feature):
                        t_start = task_solution["t_start"]
                        t_end = task_solution["t_end"]
                        agent = task_solution["agent"]
                        solution.append(actual_problem_to_solve.add_task_solution(task_id,
                                                                                  t_start,
                                                                                  t_end,
                                                                                  agent))

            makespan = max([task_sol.get_end_time() for task_sol in solution])
            synergy_index = compute_synergy_val(solution)
            show_timeline(solution)
            print("----------------------------------------")
            print(f"Recipe number: {0}")
            print(f"Recipe synerdy index: {synergy_index}")
            print(f"Makespan: {makespan}")
            print("----------------------------------------")
            # return 0
            td = TaskDispatcher(agents_group_name,
                                agents_group_name_param,
                                RECIPE_NAME.get(get_optimization_name(solution_name), "DEFAULT_NAME"))

            error_occurred_during_execution = True
            trials = 0
            # dato = input("Do you want to exectute? ")
            dato = "y"
            rospy.sleep(1)
            if "y" not in dato:
                continue
            if "stop" in dato:
                break
            while error_occurred_during_execution and trials <= 2:
                recipe_name = f"{RECIPE_NAME.get(get_optimization_name(solution_name), 'DEFAULT_NAME')}_rec_{recipe}_rep_{repetition}_{data}"
                td.set_recipe_name(recipe_name)
                print(recipe_name)

                # Publish the recipe name
                if acquire_distance:
                    pub_recipe_name.publish(String(recipe_name))

                # Dispatch the solution
                if not rospy.is_shutdown():
                    tasks_solution = add_go_home(copy.deepcopy(solution), agents_group_name)
                    td.dispatch_solution(tasks_solution)
                    trials += 1
                # Execution loop
                while not rospy.is_shutdown():
                    executed_task = td.get_performed_task()

                    # Remove from problem performed task
                    if executed_task is not None:
                        actual_problem_to_solve.remove_task(executed_task.get_task())

                    # Check if the plan failed
                    if td.is_failed_and_not_in_execution() or td.is_timeout():
                        error_occurred_during_execution = True
                        rospy.loginfo(UserMessages.PLAN_FAILED.value)
                        # Stop distance acquisition and Reload scene
                        if acquire_distance:
                            end_recipe_procedure_failure_case(recipe_name,
                                                              reset_distance_acq_srv_client,
                                                              reload_scene_srv_client)
                        else:
                            end_recipe_procedure_failure_case(recipe_name,
                                                              None,
                                                              reload_scene_srv_client)
                        td.send_recipe_end()
                        rospy.sleep(2)
                        break

                    # Check if the plan is finished
                    if td.is_finished():
                        error_occurred_during_execution = False
                        rospy.loginfo(UserMessages.PLAN_FINISHED.value)
                        # Stop distance acquisition and Reload scene
                        if acquire_distance:
                            end_recipe_procedure(stop_distance_acq_srv_client, reload_scene_srv_client)
                        else:
                            reload_scene(reload_scene_srv_client)

                        td.send_recipe_end()
                        recipe += 1
                        print("Service called and executed")

                        rospy.sleep(2)

                        break
                    rospy.sleep(1)
            input("Next...")


def end_recipe_procedure_failure_case(recipe_name,
                                      reset_distance_acq_srv_client,
                                      reload_scene_srv_client):
    delete_recipe_clnt = rospy.ServiceProxy("mongo_handler/delete_recipe", DeleteRecipe)
    recipe = DeleteRecipeRequest()
    recipe.name = recipe_name
    try:
        delete_recipe_clnt(recipe)
        rospy.loginfo(UserMessages.SERVICE_CALLBACK.value.format("recipe"))
        print("ok")
    except rospy.ServiceException as e:
        rospy.loginfo(UserMessages.SERVICE_FAILED.value.format("recipe"))

    if reset_distance_acq_srv_client:
        try:
            reset_distance_acq_srv_client()
            rospy.loginfo(UserMessages.SERVICE_CALLBACK.value.format("reset_distance_acq_srv"))
            print("ok")
        except rospy.ServiceException as e:
            rospy.loginfo(UserMessages.SERVICE_FAILED.value.format("reset_distance_acq_srv"))

    reload_scene(reload_scene_srv_client)


def end_recipe_procedure(stop_distance_acq_srv_client,
                         reload_scene_srv_client):
    stop_acquisition_distance(stop_distance_acq_srv_client)
    reload_scene(reload_scene_srv_client)


def stop_acquisition_distance(stop_distance_acq_srv_client):
    # Stop distance acquisition
    try:
        stop_distance_acq_srv_client()
        rospy.loginfo(UserMessages.SERVICE_CALLBACK.value.format("stop_distance_acq"))
    except rospy.ServiceException as e:
        rospy.loginfo(UserMessages.SERVICE_FAILED.value.format("stop_distance_acq"))


def reload_scene(reload_scene_srv_client):
    # Reload scene
    try:
        reload_scene_srv_resp = reload_scene_srv_client()
        rospy.loginfo(UserMessages.SERVICE_CALLBACK.value.format("reload_scene"))
    except rospy.ServiceException as e:
        rospy.loginfo(UserMessages.SERVICE_FAILED.value.format("reload_scene"))


if __name__ == "__main__":
    main()
