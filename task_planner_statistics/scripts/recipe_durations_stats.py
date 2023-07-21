#!/usr/bin/env python3

from MongoInterface import MongoInterface
from statistical_pipeline import StatisticalPipeline
import rospy
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import copy

RECIPE_NAMES = {"SAFETY_AREA_BOTH_AGENT_HUMAN_AWARE_EASIER": "Safety Areas-HA",
                "SAFETY_AREA_NO_AWARE": "Safety Areas-Random",
                "NOT_NEIGHBORING_TASKS_": "Safety Areas (Not Neighboring Tasks)",
                "NOT_NEIGHBORING_TASKS2_": "Safety Areas (Not Neighboring Tasks) 2",
                "ONELINE": "aware",
                "RELAXED_HA_SOLVER": "Safety Areas-HA (Relaxed)",
                "BASIC_SOLVER": "Basic TP",
                "COMPLETE_SOLVER_": "Complete HATP (30)",
                "COMPLETE_SOLVER2": "Complete HATP (50)"}
RECIPE_NAMES = {"TEST": "Safety Areas - HA",
          "SAFETY_AREA_NO_AWARE": "Safety Areas - Random",
          "NOT_NEIGHBORING_TASKS": "Not Neighboring Tasks",
          "TEST_WITH_GOHOME": "test", "ONELINE": "AWARE",
          "RELAXED_HA_SOLVER": "Relaxed HA-TP",
          "BASIC_SOLVER": "Basic TP",
          "COMPLETE_SOLVER": "Complete HA-TP"}

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

    database_name = "safety_areas"  # "milp_task_planner"
    # database_name = "velocity_scaling"  # "milp_task_planner"

    # results_collection_name = "task_results_scaling"
    # results_collection_name = "task_results_ha"

    # results_collection_name = "task_results_experiments"
    results_collection_name = "task_results_learning_phase"

    database_name = "new_safety_areas"  # "milp_task_planner"
    results_collection_name = "task_results_final_version"

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
            print(recipe_result)
        for recipe_name in RECIPE_NAMES:
            if recipe_name in recipe_result["recipe_name"]:
                recipe_result["recipe_name"] = RECIPE_NAMES[recipe_name]
                print(RECIPE_NAMES[recipe_name])
                results_base_tp.append(recipe_result)
        # if "AWARE" in recipe_result["recipe_name"]:
        #     recipe_result["recipe_name"] = "Aware Bayesian"
        #     print("Aware Bayesian")
        #     print(recipe_result)
        #     results_base_tp.append(recipe_result)
        # elif "BRUTE_FORCE" in recipe_result["recipe_name"]:
        #     recipe_result["recipe_name"] = "Brute Force"
        #     print("Brute Force")
        #     print(recipe_result)
        #     results_base_tp.append(recipe_result)
        # elif "BAD_SYNERGY" in recipe_result["recipe_name"]:
        #     recipe_result["recipe_name"] = "BAD_SYNERGY"
        #     print("BAD_SYNERGY")
        #     print(recipe_result)
        #     results_base_tp.append(recipe_result)
        # elif "RANDOM" in recipe_result["recipe_name"] and ("SCALING_RANDOM" not in recipe_result["recipe_name"]):
        #     recipe_result["recipe_name"] = "RANDOM"
        #     print("RANDOM")
        #     print(recipe_result)
        #     results_base_tp.append(recipe_result)
        # elif "SCALING_NICE" in recipe_result["recipe_name"]:
        #     recipe_result["recipe_name"] = "SCALING_NICE"
        #     print("SCALING_NICE")
        #     print(recipe_result)
        #     results_base_tp.append(recipe_result)
        # elif "SCALING_BAD" in recipe_result["recipe_name"]:
        #     recipe_result["recipe_name"] = "SCALING_BAD"
        #     print("SCALING_BAD")
        #     print(recipe_result)
        #     results_base_tp.append(recipe_result)
        # elif "SCALING_RANDOM" in recipe_result["recipe_name"]:
        #     recipe_result["recipe_name"] = "SCALING_RANDOM"
        #     print("SCALING_RANDOM")
        #     print(recipe_result)
        #     results_base_tp.append(recipe_result)
        # elif "SAFETY_AREA_BOTH_AGENT_HUMAN_AWARE_EASIER" in recipe_result["recipe_name"]:
        #     recipe_result["recipe_name"] = "SAFETY_AREA_BOTH_AGENT_HUMAN_AWARE_EASIER"
        #     print("SAFETY_AREA_BOTH_AGENT_HUMAN_AWARE_EASIER")
        #     print(recipe_result)
        #     results_base_tp.append(recipe_result)
        # elif "SAFETY_AREA_NO_AWARE" in recipe_result["recipe_name"]:
        #     recipe_result["recipe_name"] = "SAFETY_AREA_NO_AWARE"
        #     print("SAFETY_AREA_NO_AWARE")
        #     print(recipe_result)
        #     results_base_tp.append(recipe_result)
    result_pd = pd.DataFrame(results_base_tp)
    print(len(result_pd))
    sns.set_theme()

    # sns.displot(result_pd, x="recipe_duration")
    # sns.boxplot(data=result_pd, x="recipe_duration", y="recipe_name", showfliers = False )
    result_pd.rename(columns={'recipe_duration': 'Plan Duration (s)', 'recipe_name': 'Task Planner'}, inplace=True)

    sns.boxplot(data=result_pd, x="Plan Duration (s)", y="Task Planner", showfliers=False).set(
        title='Comparison of plan execution duration')
    # sns.relplot(data=result_pd["Recipe Duration (s)"])
    plt.show()


if __name__ == "__main__":
    main()
