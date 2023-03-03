#!/usr/bin/env python3

import rospy
from utils import *
from Task import Task
from Problem import Problem
from TaskPlanner import TaskPlanner
from TaskPlannerHumanAware import TaskPlannerHumanAware
from TaskPlannerHumanAwareEasier import TaskPlannerHumanAwareEasier

from TaskPlannerSynergistic import TaskPlannerSynergistic
from TaskPlannerSynergisticBand import TaskPlannerSynergisticBand
from Prove import Prove
from TaskDispatcher import TaskDispatcher
from std_srvs.srv import Trigger
from std_msgs.msg import String
import copy
import numpy as np

RECIPE_NAME = {"base": "Task_Allocation&Scheduling_Bayesian_PROVA", "human_aware": "Synergy_scaling"}


def params_exist(parameters: List[str]) -> bool:
    params_exist_check = True
    for param_name in parameters:
        if not rospy.has_param(param_name):
            rospy.logerr(UserMessages.PARAM_NOT_DEFINED_ERROR.value.format(param_name))
            params_exist_check = False
    return params_exist_check


def add_go_home(task_solution: List[TaskSolution], agents: List[str]):
    task_solution: TaskSolution
    task_solution[0].get_start_time()
    task_solution.sort(key=lambda task_sol: task_sol.get_start_time())
    task_solutions = {agent: list(filter(lambda task_sol: task_sol.get_assignment()
                                                          == agent, task_solution)) for agent in
                      agents}
    for agent in agents:
        for id, agent_task in enumerate(task_solutions[agent][:-1]):
            if task_solutions[agent][id + 1].get_start_time() > agent_task.get_end_time() + 10:
                task_solution.append(
                    TaskSolution(Task("go_home", "go_home", [agent], []),
                                 agent_task.get_end_time() + 0.1, agent_task.get_end_time() + 1, agent))
        task_solution.append(
            TaskSolution(Task("go_home", "go_home", [agent], []),
                         task_solutions[agent][-1].get_end_time() + 0.1, task_solutions[agent][-1].get_end_time() + 1,
                         agent))

    return task_solution


def compute_synergy_val(solution):
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
            synergy_index = overlapping * (synergy - 1)
            synergy_tot += synergy_index

    print(f"Synergy tot: {synergy_tot}")
    return synergy_tot


def get_best_plan(n_recipe_to_compute: int, tp: TaskPlanner):
    synergy = []
    for recipe in range(0, n_recipe_to_compute):
        solution = tp.get_solution(recipe)
        makespan = max([task_sol.get_end_time() for task_sol in solution])
        synergy.append(compute_synergy_val(tp.get_solution(recipe)))
        print(f"Recipe number: {recipe}")
        print(f"Recipe synerdy index: {synergy[recipe]}")
        print(f"Makespan: {makespan}")
        print("----------------------------------------")

    best_plan = np.argmin(synergy)
    return best_plan


def main():
    rospy.init_node("task_planner")
    parameters = ["~goal", "/agents", "/agents_group_names", "~dispatch_plan",
                  "~optimization/type", "~optimization/n_recipe", "~optimization/brute_force",
                  "~execution/repetitions", "~execution/n_recipe"]

    if not params_exist(parameters):
        return 0

    task_goal = rospy.get_param("~goal")
    agents_name = rospy.get_param("/agents")
    agents_group_name_param = rospy.get_param("/agents_group_names")

    n_recipe_to_compute = rospy.get_param("~optimization/n_recipe")
    optimization_type = rospy.get_param("~optimization/type")

    n_repetitions = rospy.get_param("~execution/repetitions")
    n_recipe_to_execute = rospy.get_param("~execution/n_recipe")

    starting_recipe_number = 0
    if not rospy.has_param("starting_recipe_number"):
        rospy.loginfo(UserMessages.PARAM_NOT_DEFINED_ERROR.value.format("starting_recipe_number"))
        rospy.loginfo("Param: starting_recipe_number set to 0")
    else:
        starting_recipe_number = rospy.get_param("starting_recipe_number")
    dispatch_plan = rospy.get_param("~dispatch_plan")

    brute_force = rospy.get_param("~optimization/brute_force")

    # rospy.wait_for_service('/reload_scene')

    # Services and Publishers
    reload_scene_srv_client = rospy.ServiceProxy('/reload_scene', Trigger)
    stop_distance_acq_srv_client = rospy.ServiceProxy("stop_distance_acq", Trigger)
    pub_recipe_name = rospy.Publisher("set_recipe_name", String, queue_size=10)

    # Agents group name ("Agent name")
    agents_group_name = []
    for agent_type in agents_name.values():
        for agent in agent_type:

            if agent not in list(agents_group_name_param.keys()):
                rospy.logerr(UserMessages.CUSTOM_RED.value.format(
                    "Param: agents_group_names, does not contain group name of agent: " + str(agent)))
                return 0
            agents_group_name.append(agents_group_name_param[agent])
    # print(agents_group_name)
    # agents_group_name = ["human_right_arm", "ur5_on_guide"]

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
        if not any(key in task_properties.keys() for key in ["task_name", "required_agents", "precedence_constraints"]):
            rospy.logerr(UserMessages.PARAM_NOT_WELL_DEFINED.value.format("goal"))
            return 0

        task_obj = Task(task_id,
                        task_properties['task_name'],
                        task_properties["required_agents"],
                        task_properties["precedence_constraints"])
        # print(task_obj)
        try:
            problem_to_solve.add_task(task_obj)
        except Exception:  # TODO: Specify Excpetion
            rospy.logerr(UserMessages.TASK_DUPLICATION.value.format(task_id))
            return 0
    if not problem_to_solve.fill_task_agents():  # TODO : Manage as exception?
        return 0
    problem_to_solve.update_tasks_statistics()
    problem_to_solve.update_tasks_synergy()
    # print(problem_to_solve)
    rospy.loginfo(f"Consistency Check: {problem_to_solve.consistency_check()}")

    # Choose the tp
    try:
        if optimization_type == "base":
            tp = TaskPlanner("Task_Allocation_Scheduling",
                             problem_to_solve,
                             objective=Objective.MAKESPAN,
                             n_solutions=n_recipe_to_compute)

        elif optimization_type == "human_aware":
            tp = Prove("tp",
                       problem_to_solve,
                       behaviour=Behaviour.CONTINUOUS,
                       objective=Objective.SYNERGY)
            # tp = TaskPlannerHumanAware("tp",
            #                            problem_to_solve,
            #                            behaviour=Behaviour.CONTINUOUS,
            #                            objective=Objective.SYNERGY)
            # tp = TaskPlannerSynergistic("Task_Allocation_Scheduling_Human_Aware",
            #                             problem_to_solve,
            #                             behaviour=Behaviour.CONTINUOUS,
            #                             objective=Objective.SYNERGY)
            # tp = TaskPlannerHumanAwareSimplified("Task_Allocation_Scheduling_Human_Aware",
            #                                      problem_to_solve,
            #                                      behaviour=Behaviour.CONTINUOUS,
            #                                      objective=Objective.ACTUAL_MAKESPAN)

            # tp = TaskPlannerSynergisticBand("tp",
            #                                 problem_to_solve,
            #                                 behaviour=Behaviour.CONTINUOUS,
            #                                 objective=Objective.SYNERGY,
            #                                 epsilon=0.2,
            #                                 relaxed=False)
            # tp = TaskPlannerHumanAware("Task_Planning&Scheduling",
            #                  problem_to_solve,
            #                    behaviour=Behaviour.CONTINUOUS,
            #                  objective=Objective.MAKESPAN)
            # tp = TaskPlannerHumanAwareEasier("Task_Planning&Scheduling",
            #                                  problem_to_solve,
            #                                  behaviour=Behaviour.CONTINUOUS,
            #                                  objective=Objective.OTHER)
    except ValueError:
        rospy.logerr(UserMessages.CONSISTENCY_CHECK_FAILED.value)
        return 0

    tp.initialize()
    tp.create_model()
    tp.add_precedence_constraints()
    if not tp.check_feasibility():
        rospy.logerr(UserMessages.PROBLEM_NOT_FEASIBLE.value.format())
        return 0
    tp.add_general_constraints()
    tp.set_objective()
    if not tp.check_feasibility():
        rospy.logerr(UserMessages.PROBLEM_NOT_FEASIBLE_DATA.value.format())
        return 0
    tp.save_model_to_file()
    tp.solve()

    actual_problem_to_solve = copy.deepcopy(problem_to_solve)


    # Compute best plan
    best_plan = None
    if brute_force and type == "base":
        best_plan = get_best_plan(n_recipe_to_compute, tp)
        # show_timeline(tp.get_solution(best_plan))
        print(f"Best plan is the number {best_plan}")

    # Exit from the loop if does not have to dispatch the plan
    if not dispatch_plan:
        return 0

    td = TaskDispatcher(agents_group_name, agents_group_name_param, RECIPE_NAME[optimization_type])

    # Iterate all task planner solution
    for recipe in range(0, n_recipe_to_execute):

        # If Brute force skip if it is not the best one
        if best_plan:
            if recipe is not best_plan:
                continue

        # Show solution
        show_timeline(tp.get_solution(recipe))
        tasks_solution = add_go_home(copy.deepcopy(tp.get_solution(recipe)), agents_group_name)
        show_timeline(tasks_solution)


        # Iterate over the execution repetitions to do
        for n_rep in range(starting_recipe_number, starting_recipe_number + n_repetitions):

            # Publish the recipe name
            pub_recipe_name.publish(String(RECIPE_NAME[optimization_type] + str(n_rep)))

            # Dispatch the solution
            td.dispatch_solution(tasks_solution)

            # Execution loop
            while not rospy.is_shutdown():
                executed_task = td.get_performed_task()

                # Remove from problem performed task
                if executed_task is not None:
                    actual_problem_to_solve.remove_task(executed_task.get_task())

                # Check if the plan failed
                if td.is_failed():
                    rospy.loginfo(UserMessages.PLAN_FAILED.value)
                    break

                # Check if the plan is finished
                if td.is_finished():
                    rospy.loginfo(UserMessages.PLAN_FINISHED.value)

                    # Stop distance acquisition
                    try:
                        stop_distance_acq_srv_client()
                        rospy.loginfo(UserMessages.SERVICE_CALLBACK.value.format("stop_distance_acq"))
                    except rospy.ServiceException as e:
                        rospy.loginfo(UserMessages.SERVICE_FAILED.value.format("stop_distance_acq"))

                    # Reload scene
                    try:
                        reload_scene_srv_resp = reload_scene_srv_client()
                        rospy.loginfo(UserMessages.SERVICE_CALLBACK.value.format("reload_scene"))
                    except rospy.ServiceException as e:
                        rospy.loginfo(UserMessages.SERVICE_FAILED.value.format("reload_scene"))

                    td.send_recipe_end()
                    print("Service called and executed")

                    rospy.sleep(5)

                    break
                rospy.sleep(1)

    #     for agent in agents:
    #         task_exec =td.get_performed_task
    #     rospy.sleep(1)
    # threads.append(threading.Thread(target=td.dispatch_solution(tp.get_solution(1)), args=tp.get_solution(1)))
    #     print("prova")
    # print("ora")
    # for thread in threads:
    #     print("dentro")
    #     thread.start()
    # td.dispatch_solution(tp.get_solution(1))
    # show_gantt(tp.get_solution())
    # rospy.loginfo(f"Consistency Check: {problem_to_solve.consistency_check()}")


if __name__ == "__main__":
    main()
