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
        if not "recipe" in recipe_result["recipe_name"]:
        #if "Task" in recipe_result["recipe_name"]:
            print(recipe_result)
            # if "Synergy" in recipe_result["recipe_name"]:
            #     recipe_result["recipe_name"] = "Synergy_TP"
            # if "Task" in recipe_result["recipe_name"] and "optim" not in recipe_result["recipe_name"]:
            #     recipe_result["recipe_name"] = "Base_TP"
            # if "opti" in recipe_result["recipe_name"]:
            #     recipe_result["recipe_name"] = "Optim"
            # if "bad" in recipe_result["recipe_name"]:
            #     recipe_result["recipe_name"] = "Bad"
            # if "BayesianBAD" in recipe_result["recipe_name"]:
            #     recipe_result["recipe_name"] = "BayesianBAD"
            #     print(recipe_result)
            #     results_base_tp.append(recipe_result)
            #
            # elif "Baye" in recipe_result["recipe_name"]:
            #     recipe_result["recipe_name"] = "Bayesian"
            #     print(recipe_result)
            #     results_base_tp.append(recipe_result)
        if "AWARE" in recipe_result["recipe_name"]:
            recipe_result["recipe_name"] = "Aware Bayesian"
            print("Aware Bayesian")
            print(recipe_result)
            results_base_tp.append(recipe_result)
        elif "BRUTE_FORCE" in recipe_result["recipe_name"]:
            recipe_result["recipe_name"] = "Brute Force"
            print("Brute Force")
            print(recipe_result)
            results_base_tp.append(recipe_result)
        elif "BAD_SYNERGY" in recipe_result["recipe_name"]:
            recipe_result["recipe_name"] = "BAD_SYNERGY"
            print("BAD_SYNERGY")
            print(recipe_result)
            results_base_tp.append(recipe_result)
        elif "RANDOM" in recipe_result["recipe_name"] and ("SCALING_RANDOM" not in recipe_result["recipe_name"]):
            recipe_result["recipe_name"] = "RANDOM"
            print("RANDOM")
            print(recipe_result)
            results_base_tp.append(recipe_result)
        elif "SCALING_NICE" in recipe_result["recipe_name"]:
            recipe_result["recipe_name"] = "SCALING_NICE"
            print("SCALING_NICE")
            print(recipe_result)
            results_base_tp.append(recipe_result)
        elif "SCALING_BAD" in recipe_result["recipe_name"]:
            recipe_result["recipe_name"] = "SCALING_BAD"
            print("SCALING_BAD")
            print(recipe_result)
            results_base_tp.append(recipe_result)
        elif "SCALING_RANDOM" in recipe_result["recipe_name"]:
            recipe_result["recipe_name"] = "SCALING_RANDOM"
            print("SCALING_RANDOM")
            print(recipe_result)
            results_base_tp.append(recipe_result)
    print(len(results_base_tp))
    result_pd = pd.DataFrame(results_base_tp)

    print(result_pd)
    sns.set_theme()

    # sns.displot(result_pd, x="recipe_duration")
    # sns.boxplot(data=result_pd, x="recipe_duration", y="recipe_name", showfliers = False )
    sns.boxplot(data=result_pd, x="recipe_duration", y="recipe_name", showfliers = False )
    plt.show()


if __name__ == "__main__":
    main()
