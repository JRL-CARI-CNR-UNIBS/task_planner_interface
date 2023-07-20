#!/usr/bin/env python3

from MongoInterface import MongoInterface
import rospy
import sys


def main():
    # rospy.init_node("Recipe_Duration_Statistics")
    # if not rospy.has_param("mongo_database"):
    #     rospy.logerr(f"Param: mongo_database not defined")
    #     return 0
    # if not rospy.has_param("mongo_collection_results"):
    #     rospy.logerr(f"Param: mongo_collection_results not defined")
    #     return 0
    #
    # database_name = rospy.get_param("mongo_database")
    # results_collection_name = rospy.get_param("mongo_collection_results")
    print(len(sys.argv))
    if len(sys.argv) < 2:
        print("No recipe name passed")
        return 0
    else:
        recipe_name = sys.argv[1]
        print(f"Recipe name: {recipe_name}")
    database_name = "milp_task_planner"
    results_collection_name = "task_results"
    mongo_interface = MongoInterface(database_name)

    results = mongo_interface.get_collection(results_collection_name)

    results.delete_many({"recipe": recipe_name})


if __name__ == "__main__":
    main()
