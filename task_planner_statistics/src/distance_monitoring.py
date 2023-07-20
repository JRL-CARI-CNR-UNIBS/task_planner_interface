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
    ovr_topic_name: str = field(init=True)

    file_path: Path = field(init=True)
    file_path_ovr: Path = field(init=True)

    window_size: int = field(default=3, init=True)

    distance_sub: rospy.Subscriber = field(init=False)
    ovr_sub: rospy.Subscriber = field(init=False)

    recipe_name: str = field(default=None, init=False)

    n_received_sample: int = field(default=0, init=False)

    window_data: np.array = field(init=False)

    timeseries_mean: np.array = field(init=False)
    timeseries_std: np.array = field(init=False)
    timeseries_ovr: np.array = field(init=False)

    timestamp_ovr: np.array = field(init=False)
    timestamp_distance: np.array = field(init=False)

    recipe_name_srv: rospy.Service = field(init=False)

    acquire: bool = field(default=False, init=False)

    def __post_init__(self):
        self.reset_window_data()
        self.timeseries_mean = np.empty(0)
        self.timeseries_std = np.empty(0)
        self.timeseries_ovr = np.empty(0)

        self.timestamp_distance = np.empty(0)
        self.timestamp_ovr = np.empty(0)

        self.distance_sub = rospy.Subscriber(self.distance_topic_name, Float32, self.update_distance)
        self.ovr_sub = rospy.Subscriber(self.ovr_topic_name, Float32, self.update_ovr)
        self.recipe_name_sub = rospy.Subscriber("set_recipe_name", String, self.update_recipe_name)

        # self.start_distance_acq_srv = rospy.Service("start_distance_acq", Trigger, self.start_distance_acq_srv)
        self.stop_distance_acq_srv = rospy.Service("stop_distance_acq", Trigger, self.stop_distance_acq_srv)
        self.reset_distance_acq_srv = rospy.Service("reset_distance_acq", Trigger, self.reset_distance_acq)

    def reset_distance_acq(self, _):
        print(f"Data distance of recipe: {self.recipe_name} erased!")
        self.reset_stats()
        response = TriggerResponse()
        response.success = True
        return response

    def reset_stats(self):
        self.acquire = False
        self.reset_window_data()

        self.timeseries_mean = np.empty(0)
        self.timeseries_std = np.empty(0)
        self.timeseries_ovr = np.empty(0)

        self.timestamp_distance = np.empty(0)
        self.timestamp_ovr = np.empty(0)

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
        if self.n_received_sample == self.window_size:
            self.timestamp_distance = np.append(self.timestamp_distance, rospy.Time.now().to_sec())
            self.timeseries_mean = np.append(self.timeseries_mean, self.window_data.mean())
            self.timeseries_std = np.append(self.timeseries_std, self.window_data.std())
            self.reset_window_data()

    def update_ovr(self, msg: Float32):
        if not self.acquire:
            print("I'm not acquiring")
            return 0
        if self.recipe_name is None:
            rospy.loginfo("Recipe name not defined")
            return 0
        self.timestamp_ovr = np.append(self.timestamp_ovr, rospy.Time.now().to_sec())
        self.timeseries_ovr = np.append(self.timeseries_ovr, msg.data)

    def update_recipe_name(self, msg):
        self.recipe_name = msg.data
        self.acquire = True
        print("Acquisition...")

    def stop_distance_acq_srv(self, req):
        print("Stop acquisition")

        n_data_distance = self.timeseries_mean.shape[0]
        timeseries_distance = pd.DataFrame({"Timestamp": self.timestamp_distance,
                                            "Mean": self.timeseries_mean,
                                            "Std": self.timeseries_std,
                                            "Recipe": [self.recipe_name] * n_data_distance})
        # print(timeseries_distance.head())
        if self.file_path.is_file():
            timeseries_distance.to_csv(self.file_path, mode='a', header=False)
        else:
            timeseries_distance.to_csv(self.file_path)
        n_data_ovr = self.timestamp_ovr.shape[0]
        timeseries_ovr = pd.DataFrame({"Timestamp": self.timestamp_ovr,
                                       "Recipe": [self.recipe_name] * n_data_ovr,
                                       "Safe_Ovr": self.timeseries_ovr})

        # print(timeseries_ovr.head())
        if self.file_path_ovr.is_file():
            timeseries_ovr.to_csv(self.file_path_ovr, mode='a', header=False)
        else:
            timeseries_ovr.to_csv(self.file_path_ovr)

        print(f"Data distance of recipe: {self.recipe_name} saved!")

        self.acquire = False
        self.reset_window_data()

        self.timeseries_mean = np.empty(0)
        self.timeseries_std = np.empty(0)
        self.timeseries_ovr = np.empty(0)

        self.timestamp_distance = np.empty(0)
        self.timestamp_ovr = np.empty(0)

        return TriggerResponse()


def main():
    rospy.init_node('distance_monitoring')
    if not rospy.has_param("~distance_monitoring_path"):
        rospy.loginfo("Param: distance_topic_name not defined")
        return 0
    if not rospy.has_param("~test_name"):
        rospy.loginfo("Param: test_name not defined")
        return 0
    test_name = rospy.get_param("~test_name")
    file_path_distance = Path(rospy.get_param("~distance_monitoring_path") + f"/distance_monitoring_{test_name}.csv")
    file_path_ovr = Path(rospy.get_param("~distance_monitoring_path") + f"/ovr_monitoring_{test_name}.csv")

    # if not rospy.has_param("distance_topic_name"):
    #     rospy.loginfo("Param: distance_topic_name not defined")
    #     return 0
    # distance_topic_name = rospy.get_param("distance_topic_name")
    distance_topic_name = "min_distance_from_poses"
    ovr_topic_name = "/safe_ovr_1_float"
    DistanceMonitoring(distance_topic_name, ovr_topic_name, file_path_distance, file_path_ovr)

    rospy.spin()


if __name__ == "__main__":
    main()
