import rospy
from utils import *
from Task import Task
from Problem import Problem


def main():
    rospy.init_node("task_planner")

    try:
        task_goal = rospy.get_param("/goal")
    except KeyError:
        rospy.logerr(UserMessages.PARAM_NOT_DEFINED_ERROR.format("goal"))
        return 0

    problem_to_solve = Problem()
    for task in task_goal:
        assert len(task.keys()) == 1
        if len(task.keys()) != 1:
            rospy.loginfo(UserMessages.PARAM_NOT_WELL_DEFINED.format("goal"))
            return 0

        task_id = list(task.keys())[0]
        task_properties = task[task_id]
        if not any(key in task_properties.keys() for key in ["task_name", "required_agents", "precedence_constraints"]):
            rospy.logerr(UserMessages.PARAM_NOT_WELL_DEFINED.format("goal"))
            return 0

        print(task_properties)
        task_obj = Task(task_id,
                        task_properties['task_name'],
                        task_properties["required_agents"],
                        task_properties["precedence_constraints"])
        problem_to_solve.add_task(task_obj)

        print(task_obj)
    rospy.loginfo(f"Consistency Check: {problem_to_solve.consistency_check()}")


if __name__ == "__main__":
    main()
