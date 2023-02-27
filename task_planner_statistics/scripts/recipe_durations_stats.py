#!/usr/bin/env python3

from MongoInterface import MongoInterface
from statistical_pipeline import StatisticalPipeline
import rospy
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import copy


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

    database_name = "milp_task_planner"
    results_collection_name = "task_results"
    mongo_interface = MongoInterface(database_name)

    pipeline = StatisticalPipeline.recipes_duration_pipeline()
    results = mongo_interface.query(results_collection_name, pipeline)

    results_base_tp = []
    for recipe_result in results:
        if not "recipe_name" in recipe_result:
            print("ERROR")
            return 0
        # print(k)
        if "Task" in recipe_result["recipe_name"]:
            print(recipe_result)
            results_base_tp.append(recipe_result)
    print(len(results_base_tp))
    result_pd = pd.DataFrame(results_base_tp)

    print(result_pd)
    sns.set_theme()

    # sns.displot(result_pd, x="recipe_duration")
    sns.boxplot(result_pd["recipe_duration"], showfliers = False)
    plt.show()


if __name__ == "__main__":
    main()
