#!/usr/bin/env python3
import seaborn

import rospy
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from pathlib import Path
import matplotlib

def main():
    sns.set_theme()
    rospy.init_node('distance_monitoring')
    if not rospy.has_param("~distance_monitoring_path"):
        rospy.loginfo("Param: distance_topic_name not defined")
        return 0

    file_path = Path(rospy.get_param("~distance_monitoring_path") + "distance_monitoring_safety_area.csv")

    distance_dataset = pd.read_csv(file_path)
    print(distance_dataset.head())
    for k in range(0, 1):
        # stringa = f"Recipe == 'SCALING_NICE{0}' or Recipe == 'SCALING_BAD{0}' or Recipe == 'SCALING_NICE{1}' or Recipe == 'SCALING_BAD{1}' or Recipe == 'SCALING_NICE{2}' or Recipe == 'SCALING_BAD{2}'"
        k = 0
        # stringa = f"Recipe == 'SCALING_NICE{k}' or Recipe == 'SCALING_BAD{k}'"
        # distance_dataset_to_show=distance_dataset.query("Recipe == 'SCALING_NICE0' or Recipe == 'SCALING_BAD0'")
        # distance_dataset_to_show = distance_dataset.query(stringa)
        # distance_dataset_to_show = distance_dataset[distance_dataset["Recipe"].str.contains("SAFETY_AREA_NO_AWARE")]
        distance_dataset_to_show = distance_dataset
        # n_bins_1 = len(distance_dataset_to_show.loc[distance_dataset_to_show['Recipe'] == "SCALING_NICE0"]["Mean"])
        # n_bins_2 = len(distance_dataset_to_show.loc[distance_dataset_to_show['Recipe'] == "SCALING_BAD0"]["Mean"])
        # n_bins = max(n_bins_1, n_bins_2)
        # count1, bins_count1 = np.histogram(
        #     distance_dataset_to_show.loc[distance_dataset_to_show['Recipe'] == "SCALING_NICE0"]["Mean"], bins=n_bins)
        # count2, bins_count2 = np.histogram(
        #     distance_dataset_to_show.loc[distance_dataset_to_show['Recipe'] == "SCALING_BAD0"]["Mean"], bins=n_bins)
        #
        # cdf1 = np.cumsum(count1) / float(n_bins_1)
        # cdf2 = np.cumsum(count2) / float(n_bins_2)
        # cdf11 = np.cumsum(count1)
        # cdf22 = np.cumsum(count2)
        #
        # min_distance = min(min(bins_count1), min(bins_count2))
        # max_distance = max(max(bins_count1), max(bins_count2))
        # d_distance = min(bins_count1[1] - bins_count1[0], bins_count2[1] - bins_count2[0])
        #
        # val_1 = []
        # val_2 = []
        # cumulative_distribution = []
        # cumulative_distribution_std = []
        # x_axis = []
        # distances = np.arange(min_distance, max_distance + d_distance, d_distance)
        # for id, distance_single in enumerate(distances):
        #     distance = distance_single
        #     x_axis.append(distance)
        #     if len(cdf1[bins_count1[1:] <= distance])>0:
        #         val_1.append(cdf1[bins_count1[1:] <= distance][-1])
        #     else:
        #         val_1.append(0)
        #     if len(cdf2[bins_count2[1:] <= distance])>0:
        #         val_2.append(cdf2[bins_count2[1:] <= distance][-1])
        #     else:
        #         val_2.append(0)
        #     cumulative_distribution.append(np.mean([val_1[id], val_2[id]]))
        #     cumulative_distribution_std.append(np.std([val_1[id], val_2[id]]))
        # lower_bound = np.array(cumulative_distribution) - np.array(cumulative_distribution_std)
        # upper_bound = np.array(cumulative_distribution) + np.array(cumulative_distribution_std)
        #
        # # plt.figure()
        #
        # fig, ax = plt.subplots()
        # ax.plot(x_axis, cumulative_distribution, '-')
        # ax.plot(bins_count1[1:], cdf1, '--')
        # ax.plot(bins_count2[1:], cdf2, '--')
        # plt.fill_between(x_axis, lower_bound, upper_bound, alpha=.3)
        # plt.show()
        #
        # print(distance_dataset_to_show.mean())
        # print(distance_dataset_to_show.axes)
        # distance_dataset[distance_dataset["Recipe"] =="SCALING_NICE_0" or distance_dataset["Recipe"] =="SCALING_BAD0"]
        # sns.set_theme()
        #
        # # sns.displot(result_pd, x="recipe_duration")
        # # sns.boxplot(data=result_pd, x="recipe_duration", y="recipe_name", showfliers = False )
        # fig, axes = plt.subplots(2, 2)

        fig = plt.figure()

        gs0 = matplotlib.gridspec.GridSpec(2, 2, figure=fig)

        ax1 = fig.add_subplot(gs0[0, 0])
        ax2 = fig.add_subplot(gs0[0, 1])
        ax3 = fig.add_subplot(gs0[1, :])
        # fig, axes = plt.subplots(nrows=1, ncols=2)
        # for axis in axes:
        #     print(type(axis))
        # return 0
        fig.suptitle('Distance analysis of running agents (H-R)')
        ax1.set_title('Density Function')
        sns.histplot(data=distance_dataset_to_show,
                     x="Mean", hue="Recipe",
                     bins=30, ax=ax1, common_norm=False, kde=True)
        ax2.set_title('Cumulative Distribution')
        prova = sns.histplot(x='Mean', data=distance_dataset_to_show, hue='Recipe', bins=len(distance_dataset_to_show),

                             stat="density",
                             element="step", fill=False, cumulative=True, common_norm=False, ax=ax2)
        ax3.set_title('Timeseries')
        # print(distance_dataset_to_show.head())
        # prova = sns.histplot(x='Mean', data=distance_dataset_to_show, hue='Recipe', bins=len(distance_dataset_to_show),
        #              stat="density",
        #              element="step", fill=False, cumulative=True, common_norm=True, ax=ax3)
        sns.lineplot(data = distance_dataset_to_show, x='Unnamed: 0', y="Mean", hue="Recipe",ax=ax3)

        # sns.lineplot(data={"nice": cdf1, "Bad": cdf2}, ax=ax3)
        # plt.figure()
        # plt.plot(bins_count1[1:], cdf1, color="red", label="SCALING_NICE0")
        # plt.plot(bins_count2[1:], cdf2, label="SCALING_BAD0")
        # plt.plot(bins_count1[1:], cdf11, color="red", label="SCALING_NICE0_nonorm")
        # plt.plot(bins_count1[1:], cdf22, color="red", label="SCALING_BAD0_nonorm")

        # plt.show()
        # fig, ax1 = plt.subplots()
        #
        # color = 'tab:red'
        # ax1.set_xlabel('time (s)')
        # ax1.set_ylabel('exp', color=color)
        # ax1.plot(bins_count1[1:], cdf1, color="red", label="SCALING_NICE0")
        # ax1.plot(bins_count2[1:], cdf2, color="blue", label="SCALING_BAD0")
        # ax1.tick_params(axis='y', labelcolor=color)
        #
        # ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
        #
        # color = 'tab:blue'
        # ax2.set_ylabel('sin', color=color)  # we already handled the x-label with ax1
        # ax2.plot(bins_count1[1:], cdf11, color="green", label="SCALING_NICE0_nonorm")
        # ax2.plot(bins_count1[1:], cdf22, color="yellow", label="SCALING_BAD0_nonorm")
        # ax2.tick_params(axis='y', labelcolor=color)
        # fig.tight_layout()  # otherwise the right y-label is slightly clipped
        plt.show()


if __name__ == "__main__":
    main()
