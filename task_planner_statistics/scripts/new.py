#!/usr/bin/env python3
import pandas
import seaborn

import rospy
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from pathlib import Path
import matplotlib
import matplotlib.ticker as mtick

ZOOM = True


def main():
    sns.set_theme()
    # rospy.init_node('distance_monitoring')
    # if not rospy.has_param("~distance_monitoring_path"):
    #     rospy.loginfo("Param: distance_monitoring_path not defined")
    #     return 0
    # if not rospy.has_param("~recipes_to_compare"):
    #     rospy.loginfo("Param: distance_topic_name not defined")
    #     return 0
    # recipes_to_compare = rospy.get_param("~recipes_to_compare")
    recipes_to_compare = ["COMPLETE_SOLVER", "RELAXED_HA", "NOT_NEIGHBORING_TASKS", "BASIC_SOLVER"]
    recipes_to_compare = recipes_to_compare[0:]
    RENAME = {"TEST": "Safety Areas - HA",
              "SAFETY_AREA_NO_AWARE": "Safety Areas - Random",
              "NOT_NEIGHBORING_TASKS": "Not Neighboring TP",
              "TEST_WITH_GOHOME": "test", "ONELINE": "AWARE",
              "RELAXED_HA_SOLVER": "Relaxed HA-TP",
              "BASIC_SOLVER": "Baseline TP",
              "COMPLETE_SOLVER": "HA-TP",
              "TEST_RELAXED": "Relaxed HA-TP",
              "RELAXED_HA":"Relaxed HA-TP"}
    RECIPE_NAME_COLUMN = "Recipe Name"
    RECIPE_TYPE_COLUMN = "Recipe Type"
    RECIPE_PERCENTAGE_COLUMN = "Percentage Under Safety Distance"
    RECIPE_S_D_TYPE_COLUMN = "Safety Distance Levels"

    risky_distances = [0.4, 0.5, 0.7, 0.8]
    # file_path = Path(rospy.get_param("~distance_monitoring_path") + "distance_monitoring_safety_area.csv")
    # file_path = "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_statistics/file/distance_monitoring_safety_area_both.csv"
    # file_path = "/home/samuele/projects/cells_ws/src/hrc_simulator/hrc_simulator/hrc_mosaic_task_planning/hrc_mosaic_task_planning_interface/statistics/distance_monitoring_safety_area.csv"

    # Era ok per aree
    file_path = "/home/samuele/projects/cells_ws/src/hrc_simulator/hrc_simulator/hrc_mosaic_task_planning/hrc_mosaic_task_planning_interface/statistics/statistiche_definitive/distance_monitoring_safety_area.csv"
    # file_path = "/home/samuele/projects/cells_ws/src/hrc_simulator/hrc_simulator/hrc_mosaic_task_planning/hrc_mosaic_task_planning_interface/statistics/distance_monitoring_velocity_scaling.csv"
    # file_path = "/home/samuele/projects/cells_ws/src/hrc_simulator/hrc_simulator/hrc_mosaic_task_planning/hrc_mosaic_task_planning_interface/statistics/distance_monitoring_safety_areas_experiments.csv"

    file_path = "/home/samuele/projects/cells_ws/src/hrc_simulator/hrc_simulator/hrc_mosaic_task_planning/hrc_mosaic_task_planning_interface/statistics/distance_monitoring_test_old_scaling.csv"
    file_path = "/home/samuele/projects/cells_ws/src/hrc_simulator/hrc_simulator/hrc_mosaic_task_planning/hrc_mosaic_task_planning_interface/statistics/new_safety_areas/distance_monitoring_new_areas_online_phase.csv"
    file_path = "/home/samuele/projects/cells_ws/src/hrc_simulator/hrc_simulator/hrc_mosaic_task_planning/hrc_mosaic_task_planning_interface/statistics/new_safety_areas/distance_monitoring_final_version.csv"
    # file_path = "/home/samuele/projects/cells_ws/src/hrc_simulator/hrc_simulator/hrc_mosaic_task_planning/hrc_mosaic_task_planning_interface/statistics/iso15066_lun_31/distance_monitoring_online_new_mar_1.csv"

    # file_path = "/home/samuele/projects/cells_ws/src/hrc_simulator/hrc_simulator/hrc_mosaic_task_planning/hrc_mosaic_task_planning_interface/statistics/iso15066_lun_31/distance_monitoring_online_new_mar_1_with_test.csv"
    # file_path = "/home/samuele/Desktop/DatiArticolo/SicurezzaAree/Distanze/distance_monitoring_final_version.csv"
    file_path = "/home/samuele/Desktop/DatiArticolo/DA MANDARE/SicurezzaContinua/results/new.csv"
    distance_dataset = pd.read_csv(file_path)
    fig, ax = plt.subplots(figsize=(16, 8))
    zoom = ZOOM
    if zoom:
        ax2 = plt.axes([0.2, 0.6, .2, .2])
        ax2.set_title('Zoom in range 0-0.8 m')
        ax2.set_ylim([0, .04])
        ax2.set_xlim([0, 1.2])
    percentage_under_risky_dataset = {RECIPE_NAME_COLUMN: [], RECIPE_TYPE_COLUMN: [], RECIPE_PERCENTAGE_COLUMN: [],
                                      RECIPE_S_D_TYPE_COLUMN: []}
    # fig, axes = plt.subplots(2, 1, sharex=True) #, figsize=(10, 5))
    min_distances = dict.fromkeys(recipes_to_compare)
    for id_type, recipe_name in enumerate(recipes_to_compare):
        single_recipe_type_data = distance_dataset[distance_dataset['Recipe'].str.contains(recipe_name)]
        single_type_recipe_names = single_recipe_type_data.Recipe.unique()

        min_distances[recipe_name] = min(single_recipe_type_data["Mean"])

        cumulative_dist_recipes = []
        bin_edges_recipes = []

        min_distance = []
        max_distance = []
        delta_distance = []

        for single_recipe in single_type_recipe_names:
            single_recipe_distance_data = distance_dataset.loc[distance_dataset['Recipe'] == single_recipe]  # ["Mean"]

            n_bins = len(single_recipe_distance_data["Mean"])

            hist, bin_edges = np.histogram(
                single_recipe_distance_data["Mean"],
                bins=n_bins)
            cumulative_dist_recipes.append(np.cumsum(hist) / float(n_bins))
            bin_edges_recipes.append(bin_edges)
            min_distance.append(min(bin_edges))
            max_distance.append(max(bin_edges))
            delta_distance.append(bin_edges[1] - bin_edges[0])

            recipe_timestamp = np.array(single_recipe_distance_data["Timestamp"])
            time_intervals = recipe_timestamp[1:] - recipe_timestamp[:-1]
            for risky_distance in risky_distances:
                tot_time_under_risky = np.sum(time_intervals[single_recipe_distance_data["Mean"][:-1] < risky_distance])
                print(f"Tot time under risky: {tot_time_under_risky}")

                percentage_time_under_risky = tot_time_under_risky / (recipe_timestamp[-1] - recipe_timestamp[0]) * 100
                print(f"Recipe duration: {recipe_timestamp[-1] - recipe_timestamp[0]}")
                print(f"Percentage time under risky distance: {percentage_time_under_risky}")
                percentage_under_risky_dataset[RECIPE_NAME_COLUMN].append(single_recipe)
                percentage_under_risky_dataset[RECIPE_TYPE_COLUMN].append(recipe_name)
                percentage_under_risky_dataset[RECIPE_PERCENTAGE_COLUMN].append(percentage_time_under_risky)
                percentage_under_risky_dataset[RECIPE_S_D_TYPE_COLUMN].append(f"Under {risky_distance} m")

        min_distance_recipes = min(min_distance)
        max_distance_recipes = max(max_distance)
        delta_distance_recipes = 0.01  # min(delta_distance)

        maximum = max(max_distance_recipes, 4.8) + delta_distance_recipes

        distances = np.arange(min_distance_recipes, maximum, delta_distance_recipes)
        x_axis = []
        cumulative_distribution = []
        cumulative_distribution_std = []

        for id, distance_single in enumerate(distances):
            distance = distance_single
            x_axis.append(distance)
            val_i = []
            for id_recipe, single_recipe in enumerate(single_type_recipe_names):
                recipe_bin_edges = bin_edges_recipes[id_recipe]
                if len(cumulative_dist_recipes[id_recipe][[recipe_bin_edges[1:] <= distance]]) > 0:
                    val_i.append(cumulative_dist_recipes[id_recipe][[recipe_bin_edges[1:] <= distance]][-1])
                else:
                    val_i.append(0)
            cumulative_distribution.append(np.mean(val_i))
            cumulative_distribution_std.append(np.std(val_i))

        lower_bound = np.array(cumulative_distribution) - 2 * np.array(cumulative_distribution_std)
        upper_bound = np.array(cumulative_distribution) + 2 * np.array(cumulative_distribution_std)
        upper_bound[upper_bound > 1] = 1
        lower_bound[lower_bound < 0] = 0

        ax.plot(x_axis, cumulative_distribution, '-', label=f"{RENAME[recipe_name]}")

        ax.fill_between(x_axis, lower_bound, upper_bound, alpha=.15, label="(Confidence 95 %)")
        handles, labels = ax.get_legend_handles_labels()

        ax.legend(handles=[(h1, h2) for h1, h2 in zip(handles[::2], handles[1::2])],
                  labels=[l1 + " " + l2 for l1, l2 in zip(labels[::2], labels[1::2])], loc='lower right', fontsize=20)
        ax.figure.set_size_inches(17, 10)
        for item in ([ax.title, ax.xaxis.label, ax.yaxis.label] +
                     ax.get_xticklabels() + ax.get_yticklabels()):
            item.set_fontsize(20)

    print(min_distances)

    min_distances_pd = pd.DataFrame(min_distances.items(), columns=['Method', 'Min Distance (m)'])
    print(min_distances_pd)

    min_distances_pd["Method"] = min_distances_pd["Method"].replace(RENAME)

    baseline_distance = min_distances_pd[min_distances_pd['Method'] == 'Baseline TP']['Min Distance (m)'].values[0]
    min_distances_pd['Difference from Baseline (m)'] =  min_distances_pd['Min Distance (m)'] - baseline_distance
    not_neigh_distance = min_distances_pd[min_distances_pd['Method'] == 'Not Neighboring TP']['Min Distance (m)'].values[0]
    min_distances_pd['Difference from Not Neighboring (m)'] =  min_distances_pd['Min Distance (m)'] - not_neigh_distance
    latex_table = min_distances_pd.to_latex(index=False)
    from tabulate import tabulate
    latex_table = tabulate(min_distances_pd, headers="keys", tablefmt="latex_raw")

    print(min_distances_pd)

    print(latex_table)
    # return 0
    # fdsfds
    # return 0
    # plt.xlabel("Minimum Human-Robot Distance (m)", labelpad=25)
    # plt.ylabel("Cumulative Distribution Function")
    ax.set_xlabel("Minimum Human-Robot Distance (m)", labelpad=25)  # Imposta il nome dell'asse x per ax
    ax.set_ylabel("Cumulative Distribution Function")  # Imposta il nome dell'asse y per ax
    # import plotly.tools as tls
    # from plotly.offline import download_plotlyjs, init_notebook_mode, iplot
    # plotly_fig = tls.mpl_to_plotly(fig)  ## convert
    # iplot(plotly_fig)
    # plt.gca().yaxis.set_major_formatter(mtick.PercentFormatter(xmax=1))

    if zoom:
        for id_type, recipe_name in enumerate(recipes_to_compare):
            single_recipe_type_data = distance_dataset[distance_dataset['Recipe'].str.contains(recipe_name)]
            single_type_recipe_names = single_recipe_type_data.Recipe.unique()

            cumulative_dist_recipes = []
            bin_edges_recipes = []

            min_distance = []
            max_distance = []
            delta_distance = []

            for single_recipe in single_type_recipe_names:
                single_recipe_distance_data = distance_dataset.loc[
                    distance_dataset['Recipe'] == single_recipe]  # ["Mean"]

                n_bins = len(single_recipe_distance_data["Mean"])

                hist, bin_edges = np.histogram(
                    single_recipe_distance_data["Mean"],
                    bins=n_bins)
                cumulative_dist_recipes.append(np.cumsum(hist) / float(n_bins))
                bin_edges_recipes.append(bin_edges)
                min_distance.append(min(bin_edges))
                max_distance.append(max(bin_edges))
                delta_distance.append(bin_edges[1] - bin_edges[0])

                recipe_timestamp = np.array(single_recipe_distance_data["Timestamp"])
                time_intervals = recipe_timestamp[1:] - recipe_timestamp[:-1]
                for risky_distance in risky_distances:
                    tot_time_under_risky = np.sum(
                        time_intervals[single_recipe_distance_data["Mean"][:-1] < risky_distance])
                    print(f"Tot time under risky: {tot_time_under_risky}")

                    percentage_time_under_risky = tot_time_under_risky / (
                            recipe_timestamp[-1] - recipe_timestamp[0]) * 100
                    print(f"Recipe duration: {recipe_timestamp[-1] - recipe_timestamp[0]}")
                    print(f"Percentage time under risky distance: {percentage_time_under_risky}")
                    percentage_under_risky_dataset[RECIPE_NAME_COLUMN].append(single_recipe)
                    percentage_under_risky_dataset[RECIPE_TYPE_COLUMN].append(recipe_name)
                    percentage_under_risky_dataset[RECIPE_PERCENTAGE_COLUMN].append(percentage_time_under_risky)
                    percentage_under_risky_dataset[RECIPE_S_D_TYPE_COLUMN].append(f"Under {risky_distance} m")

            min_distance_recipes = min(min_distance)
            max_distance_recipes = max(max_distance)
            delta_distance_recipes = 0.01  # min(delta_distance)

            maximum = max(max_distance_recipes, 4.5) + delta_distance_recipes

            distances = np.arange(min_distance_recipes, maximum, delta_distance_recipes)
            x_axis = []
            cumulative_distribution = []
            cumulative_distribution_std = []

            for id, distance_single in enumerate(distances):
                distance = distance_single
                x_axis.append(distance)
                val_i = []
                for id_recipe, single_recipe in enumerate(single_type_recipe_names):
                    recipe_bin_edges = bin_edges_recipes[id_recipe]
                    if len(cumulative_dist_recipes[id_recipe][[recipe_bin_edges[1:] <= distance]]) > 0:
                        val_i.append(cumulative_dist_recipes[id_recipe][[recipe_bin_edges[1:] <= distance]][-1])
                    else:
                        val_i.append(0)
                cumulative_distribution.append(np.mean(val_i))
                cumulative_distribution_std.append(np.std(val_i))

            lower_bound = np.array(cumulative_distribution) - 2 * np.array(cumulative_distribution_std)
            upper_bound = np.array(cumulative_distribution) + 2 * np.array(cumulative_distribution_std)
            upper_bound[upper_bound > 1] = 1
            lower_bound[lower_bound < 0] = 0

            ax2.plot(x_axis, cumulative_distribution, '-')

            # ax2.fill_between(x_axis, lower_bound, upper_bound, alpha=.15)
            handles, labels = ax.get_legend_handles_labels()

            for item in ([ax.title, ax.xaxis.label, ax.yaxis.label] +
                         ax.get_xticklabels() + ax.get_yticklabels()):
                item.set_fontsize(20)
            # sns.lineplot(x_axis, cumulative_distribution, ax=ax2)
        # plt.gca().yaxis.set_major_formatter(mtick.PercentFormatter(xmax=1, decimals=0))
        ax2.yaxis.set_major_formatter(mtick.PercentFormatter(xmax=1, decimals=0))

        # ax.figure.set_size_inches(22, 8)
    font = {'size': 50}

    # ax2.set_xlabel('')  # Rimuove il nome dell'asse x
    # ax2.set_ylabel('')  # Rimuove il nome dell'asse y


    # plt.title("Comparison on cumulative probability density on minimum H-R distance")
    # plt.axvline(x=0.8, ymin=0, ymax=1, ls='--', color="#636E72")
    # plt.annotate(xy=(0, 0.8), xytext=(0.8, 0.8), arrowprops=dict(arrowstyle='<|-|>', color="#636E72", lw=1.5), text="")
    # plt.annotate(xy=(0.08, 0.81), text="RISKY H-R DISTANCE (0.8 m) ")

    path = "/home/samuele/projects/cells_ws/src/hrc_simulator/hrc_simulator/hrc_mosaic_task_planning/hrc_mosaic_task_planning_interface/statistics/fig/iso15066/new_version_25_ago/"
    path = "/home/samuele/projects/cells_ws/src/hrc_simulator/hrc_simulator/hrc_mosaic_task_planning/hrc_mosaic_task_planning_interface/statistics/fig/iso15066/new_version_25_ago/"

    # plt.title("Comparison of plan execution duration", pad=20)
    # plt.ylabel("Task Planner Type", labelpad=25)
    # plt.xlim(75, 105)

    # Formattazione dell'asse y esterno
    ax.yaxis.set_major_formatter(mtick.PercentFormatter(xmax=1, decimals=0))

    zoom_name = ""
    if zoom:
        zoom_name = "_with_zoom"
    # plt.savefig(f"{path}comparison_min_distance_higher_test_prova_smaller_{zoom_name}.png", bbox_inches='tight')
    # plt.savefig(f"{path}comparison_min_distance_higher_test_prova_smaller_{zoom_name}.pdf", bbox_inches='tight')

    # sns.set(rc={'figure.figsize': (25,8)})

    # plt.savefig(f"{path}comparison_min_distance{zoom_name}.png", bbox_inches='tight')
    # plt.savefig(f"{path}comparison_min_distance{zoom_name}.pdf", bbox_inches='tight')

    percentage_under_risky_dataset_pd = pandas.DataFrame(percentage_under_risky_dataset)

    # fig, ax = plt.subplots(figsize=(16, 8))
    percentage_under_risky_dataset_pd = percentage_under_risky_dataset_pd.rename(columns=RENAME)
    print(percentage_under_risky_dataset_pd.head())
    sns.catplot(data=percentage_under_risky_dataset_pd, kind="bar", x=RECIPE_S_D_TYPE_COLUMN,
                y=RECIPE_PERCENTAGE_COLUMN, hue=RECIPE_TYPE_COLUMN, height=8, aspect=1.5)
    # sns.set(rc={'figure.figsize': (11.7, 8.27)})

    # plt.gca().yaxis.set_major_formatter(mtick.PercentFormatter())
    plt.title("Safety Distance Comparison")

    plt.show()
    print(min_distances)
    # min_distances_pd = pd.DataFrame(min_distances)
    # min_distances_pd.rename(columns=RENAME)
    # print(min_distances_pd.head())


if __name__ == "__main__":
    main()