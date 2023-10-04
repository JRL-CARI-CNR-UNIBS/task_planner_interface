#!/usr/bin/env python3
from dataclasses import dataclass, field
from pathlib import Path

from typing import Dict

import rospy
import std_msgs.msg
from std_msgs.msg import Float32, String, Bool
from std_srvs.srv import Trigger, TriggerResponse

from task_planner_interface_msgs.msg import MotionTaskExecutionRequest, \
    MotionTaskExecutionRequestArray, \
    MotionTaskExecutionFeedback

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

    task_feedback_sub: Dict[str, rospy.Subscriber] = field(init=False)
    task_request_sub: Dict[str, rospy.Subscriber] = field(init=False)

    task_in_execution: Dict[str,str] = field(init=False)

    timeseries_task_ovr: Dict[str, np.array] = field(init=False)
    timeseries_task_distance: Dict[str, np.array] = field(init=False)

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

        if True:
            # TODO: Get Param
            agents_topic_name = {"request": {"manipulator": "/sharework/test/stiima/motion",
                                             "human": "/sharework/test/stiima/human"},
                                 "feedback": {"manipulator": "/sharework/test/stiima/motion/feedback",
                                              "human": "/sharework/test/stiima/human/feedback"}}
            agents = None
            if sorted(agents_topic_name["request"].keys()) == sorted(
                    agents_topic_name["feedback"].keys()):
                agents = agents_topic_name["request"].keys()
            else:
                raise KeyError("Task Feedback and Task request must have same agents to subscribe")
            self.task_feedback_sub = dict.fromkeys(agents, None)
            self.task_request_sub = dict.fromkeys(agents, None)
            for agent in agents:
                feedback_topic_name = agents_topic_name["feedback"][agent]
                request_topic_name = agents_topic_name["request"][agent]
                self.task_feedback_sub[agent] = rospy.Subscriber(feedback_topic_name,
                                                                 MotionTaskExecutionFeedback,
                                                                 self.reset_task_in_execution, agent)
                self.task_request_sub[agent] = rospy.Subscriber(request_topic_name,
                                                                MotionTaskExecutionRequestArray,
                                                                self.set_task_in_execution, agent)

            self.timeseries_task_ovr = dict.fromkeys(agents, np.empty(0, dtype=str))
            self.timeseries_task_distance = dict.fromkeys(agents, np.empty(0, dtype=str))
            self.task_in_execution = dict.fromkeys(agents, "")

        # self.start_distance_acq_srv = rospy.Service("start_distance_acq", Trigger, self.start_distance_acq_srv)
        self.stop_distance_acq_srv = rospy.Service("stop_distance_acq", Trigger, self.stop_distance_acq_srv)
        self.reset_distance_acq_srv = rospy.Service("reset_distance_acq", Trigger, self.reset_distance_acq)

    def set_task_in_execution(self, msg: MotionTaskExecutionRequestArray, agent: str):
        self.task_in_execution[agent] = msg.tasks[0].task_id

    def reset_task_in_execution(self, _, agent: str):
        self.task_in_execution[agent] = ""
        print(f"TASK: {self.task_in_execution}")

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
        self.timeseries_task_ovr = dict.fromkeys(self.timeseries_task_ovr.keys(), np.empty(0, dtype=str))
        self.timeseries_task_distance = dict.fromkeys(self.timeseries_task_distance.keys(), np.empty(0, dtype=str))

        self.timestamp_distance = np.empty(0)
        self.timestamp_ovr = np.empty(0)

        for agent in self.task_in_execution:
            self.task_in_execution[agent]  = ""

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
        # print(self.timeseries_task_distance)
        self.window_data[self.n_received_sample] = msg.data
        self.n_received_sample += 1
        if self.n_received_sample == self.window_size:
            self.timestamp_distance = np.append(self.timestamp_distance, rospy.Time.now().to_sec())
            self.timeseries_mean = np.append(self.timeseries_mean, self.window_data.mean())
            self.timeseries_std = np.append(self.timeseries_std, self.window_data.std())
            for agent in self.timeseries_task_distance:
                self.timeseries_task_distance[agent] = np.append(self.timeseries_task_distance[agent],
                                                                 self.task_in_execution[agent])

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

        for agent in self.timeseries_task_ovr:
            self.timeseries_task_ovr[agent] = np.append(self.timeseries_task_ovr[agent],
                                                        self.task_in_execution[agent])

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
        for agent in self.task_in_execution:
            # print(self.timeseries_task_distance)
            timeseries_distance.insert(2, agent, self.timeseries_task_distance[agent], True)
        # print(timeseries_distance.head())
        if self.file_path.is_file():
            timeseries_distance.to_csv(self.file_path, mode='a', header=False)
        else:
            timeseries_distance.to_csv(self.file_path)
        n_data_ovr = self.timestamp_ovr.shape[0]
        timeseries_ovr = pd.DataFrame({"Timestamp": self.timestamp_ovr,
                                       "Recipe": [self.recipe_name] * n_data_ovr,
                                       "Safe_Ovr": self.timeseries_ovr})
        for agent in self.task_in_execution:
            timeseries_ovr.insert(2, agent, self.timeseries_task_ovr[agent], True)
            print(self.timeseries_task_ovr)
        print(timeseries_ovr)
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
        self.timeseries_task_ovr = dict.fromkeys(self.timeseries_task_ovr.keys(), np.empty(0, dtype=str))
        self.timeseries_task_distance = dict.fromkeys(self.timeseries_task_distance.keys(), np.empty(0, dtype=str))

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
