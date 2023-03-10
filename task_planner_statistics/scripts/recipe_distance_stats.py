#!/usr/bin/env python3

import rospy
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from pathlib import Path


def main():
    sns.set_theme()
    rospy.init_node('distance_monitoring')
    if not rospy.has_param("~distance_monitoring_path"):
        rospy.loginfo("Param: distance_topic_name not defined")
        return 0

    file_path = Path(rospy.get_param("~distance_monitoring_path") + "distance_monitoring.csv")

    distance_dataset = pd.read_csv(file_path)
    print(distance_dataset.head())
    for k in range(0, 3):
        stringa = f"Recipe == 'SCALING_NICE{k}' or Recipe == 'SCALING_BAD{k}'"
        # distance_dataset_to_show=distance_dataset.query("Recipe == 'SCALING_NICE0' or Recipe == 'SCALING_BAD0'")
        distance_dataset_to_show = distance_dataset.query(stringa)

        print(distance_dataset_to_show.mean())
        # print(distance_dataset_to_show.axes)
        # distance_dataset[distance_dataset["Recipe"] =="SCALING_NICE_0" or distance_dataset["Recipe"] =="SCALING_BAD0"]
        # sns.set_theme()
        #
        # # sns.displot(result_pd, x="recipe_duration")
        # # sns.boxplot(data=result_pd, x="recipe_duration", y="recipe_name", showfliers = False )
        # fig, axes = plt.subplots(2, 2)
        import matplotlib
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
                     bins=30, ax=ax1, common_norm=False,kde=True)
        ax2.set_title('Cumulative Distribution')
        sns.histplot(x='Mean', data=distance_dataset_to_show, hue='Recipe', bins=len(distance_dataset_to_show),
                     stat="density",
                     element="step", fill=False, cumulative=True, common_norm=False, ax=ax2)
        ax3.set_title('Timeseries')
        # print(distance_dataset_to_show.head())
        sns.lineplot(data = distance_dataset_to_show, x='Unnamed: 0', y="Mean", hue="Recipe",ax=ax3)

        plt.show()


if __name__ == "__main__":
    main()
