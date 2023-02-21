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


def main():
    rospy.init_node("task_planner")

    try:
        task_goal = rospy.get_param("/goal")
    except KeyError:
        rospy.logerr(UserMessages.PARAM_NOT_DEFINED_ERROR.value.format("goal"))
        return 0

    problem_to_solve = Problem(["human_right_arm", "ur5_on_guide"])

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
    rospy.loginfo(f"Consistency Check: {problem_to_solve.consistency_check()}")
    try:
        tp = TaskPlanner("Task_Planning&Scheduling",
                                         problem_to_solve,
                         n_solutions=10)
        # tp = TaskPlannerHumanAware("Task_Planning&Scheduling",
        #                                  problem_to_solve,
        #                                  behaviour=Behaviour.CONTINUOUS,
        #                                  objective=Objective.SYNERGY)
        # tp = TaskPlannerSynergisticBand("Task_Planning&Scheduling",
        #                                  problem_to_solve,
        #                                  behaviour=Behaviour.CONTINUOUS,
        #                                  objective=Objective.MAKESPAN)
        # tp = TaskPlannerSynergistic("Task_Planning&Scheduling",
        #                             problem_to_solve,
        #                             behaviour=Behaviour.CONTINUOUS,
        #                             objective=Objective.SYNERGY)

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
    tp.solve()
    # print(tp.get_solution())
    for k in range(10):
        show_timeline(tp.get_solution(k))
    # show_gantt(tp.get_solution())
    # rospy.loginfo(f"Consistency Check: {problem_to_solve.consistency_check()}")


if __name__ == "__main__":
    main()
