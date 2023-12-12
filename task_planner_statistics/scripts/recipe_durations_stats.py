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
RECIPE_NAMES = {"TESTdsfds": "Safety Areas - HA",
                "SAFETY_AREA_NO_AWARE": "Safety Areas - Random",
                "NOT_NEIGHBORING_TASKS": "Not Neighboring TP",
                "TEST_WITH_GOHOME": "test", "ONELINE": "AWARE",
                "RELAXED_HA_SOLVER": "HA-TP (Relaxed)",
                "BASIC_SOLVER": "Baseline TP",
                "COMPLETE_SOLVER": "HA-TP",
                "COMPLETE_HA_SOLVER": "HA-TP",
                "TEST_COMPLETE": "Test",
                "TEST_RELAXED": "Test REL",
                "NEW": "New"
                }


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

    # TODO: QUESTO E' OK PER LE AREE
    # database_name = "new_safety_areas"  # "milp_task_planner"
    # results_collection_name = "task_results_final_version"

    # database_name = "iso15066"  # "milp_task_planner"
    # results_collection_name = "task_results_online_more_tasks"

    # Ultimo
    database_name = "safety_areas_7_ago"  # "milp_task_planner"
    results_collection_name = "task_results_25_ago"
    # database_name = "iso15066_lun_31"  # "milp_task_planner"
    # results_collection_name = "complete_task_results"


    database_name = "hrc_case_study"
    results_collection_name = "real_test_results_test_75_plans"
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
                if "TEST" in recipe_result["recipe_name"]:
                    recipe_result["recipe_name"] = "Test"
                    results_base_tp.append(recipe_result)
                    continue
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
    result_pd.rename(columns={'recipe_duration': 'Plan Duration (s)', 'recipe_name': 'Task Planner Type'}, inplace=True)

    ax = sns.boxplot(data=result_pd, x="Plan Duration (s)", y="Task Planner Type", showfliers=False, order= ["Baseline TP",
                            "Not Neighboring TP",
                            "HA-TP (Relaxed)",
                            "HA-TP"])
    from tabulate import tabulate

    # # Calcola le medie per ciascun "Task Planner Type"
    mean_duration = result_pd.groupby("Task Planner Type")["Plan Duration (s)"].mean().reset_index()

    # Rinomina la colonna delle medie
    mean_duration.rename(columns={"Plan Duration (s)": "Mean Duration (s)"}, inplace=True)


    # Trova il valore per "baseline tp" e "not neighboring tp"
    baseline_duration = mean_duration[mean_duration["Task Planner Type"] == "Baseline TP"]["Mean Duration (s)"].values[
        0]

    not_neighboring_duration = \
    mean_duration[mean_duration["Task Planner Type"] == "Not Neighboring TP"]["Mean Duration (s)"].values[0]

    # Calcola la riduzione percentuale rispetto a "baseline tp" e "not neighboring tp"
    mean_duration["Reduction from Baseline (%)"] = ((baseline_duration - mean_duration[
        "Mean Duration (s)"]) / baseline_duration) * 100
    mean_duration["Reduction from Not Neighboring (%)"] = ((not_neighboring_duration - mean_duration[
        "Mean Duration (s)"]) / not_neighboring_duration) * 100

    # Converte il DataFrame delle medie in una tabella LaTeX
    latex_table = tabulate(mean_duration, headers="keys", tablefmt="latex_raw")
    # Stampa la tabella LaTeX
    print(latex_table)
    # return 0
    # # print(mean_duration)
    # order=(["Baseline TP",
    #                         "Not Neighboring TP",
    #                         "HA-TP (Relaxed)",
    #                         "HA-TP"])
    #
    import plotly.express as px
    fig = px.box(result_pd, x="Plan Duration (s)", y="Task Planner Type", color="Task Planner Type")
    fig.show()

    # import plotly.io as pio
    # pio.write_html(fig, file='/home/samuele/Desktop/DatiArticolo/Definitivi/SicurezzaContinua/figure.html', auto_open=True)
    # print(pio.write_html(fig, file='/home/samuele/Desktop/DatiArticolo/Definitivi/SicurezzaContinua/figure.html', auto_open=True))
    # return 0

    # sns.set(y_label)
    # sns.relplot(data=result_pd["Plan Duration (s)"])
    path = "/home/samuele/projects/cells_ws/src/hrc_simulator/hrc_simulator/hrc_mosaic_task_planning/hrc_mosaic_task_planning_interface/statistics/fig/safety_areas/"
    path = "/home/samuele/projects/cells_ws/src/hrc_simulator/hrc_simulator/hrc_mosaic_task_planning/hrc_mosaic_task_planning_interface/statistics/fig/iso15066/new_version_25_ago/duration/"
    # path = "/home/samuele/Desktop/DatiArticolo/Definitivi/LessTaskSafetyAreas/grafici/"
    # fig.write_html(f"{path}duration.html")
    #
    # #
    # plt.title("Comparison of plan execution duration", pad=20)
    # plt.ylabel("Task Planner Type", labelpad=25)
    # plt.xlim(77, 92)
    # plt.savefig(f"{path}iso15066_comparison_plan_duration_with_label.png", bbox_inches='tight')
    # plt.savefig(f"{path}iso15066_comparison_plan_duration_with_label.pdf", bbox_inches='tight')
    # #
    # plt.title("")
    # plt.ylabel("")
    # plt.savefig(f"{path}iso15066_comparison_plan_duration.png", bbox_inches='tight')
    # plt.savefig(f"{path}iso15066_comparison_plan_duration.pdf", bbox_inches='tight')
    #
    # # plt.axis.set_size_inches(17, 8)
    # ax.figure.set_size_inches(17, 8)
    # # sns.set(rc={'figure.figsize': (25,8)})
    #
    # plt.savefig(f"{path}_iso15066_comparison_plan_duration_.png", bbox_inches='tight')

    path = "/home/galois/Desktop/Samuele/"
    plt.ylabel("")
    plt.savefig(f"{path}iso15066_comparison_plan_duration_test_finished.pdf", bbox_inches='tight')
    plt.show()


if __name__ == "__main__":
    main()
