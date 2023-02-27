#!/usr/bin/env python3
from dataclasses import dataclass, field
from pathlib import Path

import rospy
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger, TriggerResponse
import numpy as np
import pandas as pd


@dataclass
class DistanceMonitoring:
    distance_topic_name: str = field(init=True)
    file_path: Path = field(init=True)

    distance_sub: rospy.Subscriber = field(init=False)
    recipe_name: str = field(default=None, init=False)

    n_received_sample: int = field(default=0, init=False)

    window_data: np.array = field(init=False)

    timeseries_mean: np.array = field(init=False)
    timeseries_std: np.array = field(init=False)

    recipe_name_srv: rospy.Service = field(init=False)

    acquire: bool = field(default=False, init=False)
    window_size: int = field(default=5, init=True)

    def __post_init__(self):
        self.reset_window_data()
        self.timeseries_mean = np.empty(0)
        self.timeseries_std = np.empty(0)
        self.distance_sub = rospy.Subscriber(self.distance_topic_name, Float32, self.update_distance)
        self.recipe_name_sub = rospy.Subscriber("set_recipe_name", String, self.update_recipe_name)

        # self.start_distance_acq_srv = rospy.Service("start_distance_acq", Trigger, self.start_distance_acq_srv)
        self.stop_distance_acq_srv = rospy.Service("stop_distance_acq", Trigger, self.stop_distance_acq_srv)

    def reset_window_data(self):
        self.n_received_sample = 0
        self.window_data = np.zeros(self.window_size)

    def update_distance(self, msg: Float32):
        if not self.acquire:
            print("I'm not acquiring")
            return 0
        if self.recipe_name is None:
            rospy.loginfo("Recipe name not defined")
            return 0
        self.window_data[self.n_received_sample] = msg.data
        self.n_received_sample += 1
        print(self.n_received_sample)
        if self.n_received_sample == self.window_size:
            self.timeseries_mean = np.append(self.timeseries_mean, self.window_data.mean())
            self.timeseries_std = np.append(self.timeseries_std, self.window_data.std())
            self.reset_window_data()

    def update_recipe_name(self, msg):
        self.recipe_name = msg.data
        self.acquire = True
        print("Acquisition...")

    def stop_distance_acq_srv(self, req):
        print("Stop acquisition")
        n_data = self.timeseries_mean.shape[0]
        timeseries = pd.DataFrame({"Mean": self.timeseries_mean,
                                   "Std": self.timeseries_std,
                                   "Recipe": [self.recipe_name] * n_data})
        print(timeseries.head())
        if self.file_path.is_file():
            timeseries.to_csv(self.file_path, mode='a', header=False)
        else:
            timeseries.to_csv(self.file_path)

        self.acquire = False
        self.reset_window_data()
        self.timeseries_mean = np.empty(0)
        self.timeseries_std = np.empty(0)
        return TriggerResponse()


def main():
    rospy.init_node('distance_monitoring')
    if not rospy.has_param("~distance_monitoring_path"):
        rospy.loginfo("Param: distance_topic_name not defined")
        return 0

    file_path = Path(rospy.get_param("~distance_monitoring_path") + "distance_monitoring.csv")

    # if not rospy.has_param("distance_topic_name"):
    #     rospy.loginfo("Param: distance_topic_name not defined")
    #     return 0
    # distance_topic_name = rospy.get_param("distance_topic_name")
    distance_topic_name = "min_distance_from_poses"
    DistanceMonitoring(distance_topic_name, file_path)

    # rate = rospy.Rate(30)
    # while not rospy.is_shutdown():
    #     rospy.spinOnce
    #     rate.sleep()
    rospy.spin()


if __name__ == "__main__":
    main()
