#! /usr/bin/env python3

import rospy

from std_srvs.srv import SetBool, SetBoolResponse
from Pipeline import Pipeline

from pymongo import MongoClient
import pymongo.errors

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

import numpy as np
import pprint
import os
import copy

import pandas as pd
import seaborn as sns
# import statsmodels.api as sm
import statsmodels.formula.api as smf
from sklearn.ensemble import IsolationForest

import pyro
import pyro.distributions as dist
import pyro.optim as optim
from pyro.infer import MCMC, NUTS
import torch

from task_planner_statistics.msg import TaskSynergy
from task_planner_statistics.srv import TaskSynergies, TaskSynergiesResponse

import multiprocessing

import yaml
import pickle

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'

SERVICE_CALLBACK = GREEN + "Service call {} received" + END
READY = GREEN + "Ready to manage db" + END
PARAM_NOT_DEFINED_ERROR = "Parameter: {} not defined"
CONNECTION_FAILED = "Connection to db failed: Request Timeout"
CONNECTION_OK = GREEN + "Connection to db executed" + END
SUCCESSFUL = "Successfully executed"
NOT_SUCCESSFUL = "Not Successfully executed"
CONNECTION_LOST = RED + "Connection to Database lost" + END
DURATION_OK = "Duration computed correctly"

COMPUTE_DURATION_SERVICE = "mongo_statistics/compute_durations"
COMPUTE_DYNAMIC_RISK_SERVICE = "mongo_statistics/compute_dynamic_risk"
COMPUTE_DYNAMIC_RISK_WITH_UNC_SERVICE = "mongo_statistics/compute_dynamic_risk_unc"
COMPUTE_BAYESIAN_DYNAMIC_RISK_SERVICE = "mongo_statistics/compute_bayesian_dynamic_risk"

TASK_DURATION_CHART_SERVICE = "mongo_statistics/task_duration_chart"
TASK_DURATION_BY_GROUPING_CHART_SERVICE = "mongo_statistics/task_duration_by_grouping_chart"
SYNERGY_MATRIX_CHART_SERVICE = "mongo_statistics/synergy_matrix_chart"
SYNERGY_UNCERTAINTY_CHART_SERVICE = "mongo_statistics/synergy_uncertainty_chart"
TIMELINE_CHART_SERVICE = "mongo_statistics/timeline_chart"

PARTIAL_INITIAL = "partial_task_initial"
PARTIAL_FINAL = "partial_task_final"
INNER = "inner_task"
OUTER = "outer_task"

paper = False

AGENT_NAME_FOR_CHART = {"ur5_on_guide": "Robot", "human_right_arm": "Human"}
AGENT_NAME_FOR_CHART = {"manipulator": "Robot", "human": "Human"}

TASK_NAME_FOR_CHART = {"pick_orange_box": "Pick orange box",
                       "place_orange_box": "Place orange box",
                       "pick_blue_box": "Pick blue Box",
                       "place_blue_box_ur5_on_guide": "Place blue box (R)",
                       "pick_white_box": "Pick white box",
                       "place_white_box": "Place white box",
                       "place_blue_box_human_right_arm": "Place blue box (H)"
                       }

TASK_NAME_FOR_CHART = {"battery_sorting": "Battery Sorting",
                       "board_localization": "Board Localization",
                       "disassemble_remote_control": "disassemble_remote_control",
                       "probe": "probe",
                       "remove_notebook_components": "remove_notebook_components",
                       "remove_notebook_cover": "remove_notebook_cover",
                       "remove_plug": "remove_plug"
                       }


class MongoStatistics:

    def __init__(self, db_name, coll_properties_name, coll_results_name, coll_duration_name, coll_risk_name,
                 results_folder_path):
        """Constructor

        Args:
            db_name (string): Database name
            coll_properties_name (string): Collection name with task properties
            coll_results_name (string): Collection name in which saving task results
            coll_duration_name (string): Collection name in which save "single-agents" task statistical information
            coll_risk_name (string): Collection name in which save elements of dynamic risk matrix

        Returns:
            string: Collection name
        """

        client = MongoClient(serverSelectionTimeoutMS=5000)  # 5 seconds of maximum connection wait
        try:
            client.server_info()
            rospy.loginfo(GREEN + CONNECTION_OK + END)
        except pymongo.errors.ServerSelectionTimeoutError:
            rospy.logerr(RED + CONNECTION_FAILED + END)
            raise

        if db_name not in client.list_database_names():
            rospy.loginfo(RED + "The specified db does not exist. A db with empty collections will be created. " + END)
        self.db = client[db_name]

        self.coll_skills = self.db[coll_properties_name]
        self.coll_results = self.db[coll_results_name]
        self.coll_interaction = self.db[coll_risk_name]
        self.coll_durations = self.db[coll_duration_name]

        self.coll_durations_name = coll_duration_name
        self.coll_results_name = coll_results_name

        self.utils_results_name = "utils_task_results"

        self.coll_utils_results = self.db[self.utils_results_name]

        self.results_folder_path = results_folder_path

    def computeDurations(self, request):
        """Method to compute the expected duration of each task with standard deviation, min and max information.

        Args:
            request (_type_): _description_

        Returns:
            _type_: _description_
        """

        rospy.loginfo(SERVICE_CALLBACK.format(COMPUTE_DURATION_SERVICE))

        pipeline = Pipeline.durationComputationPipeline(self.coll_durations_name)

        try:
            self.coll_durations.delete_many({})
            results = self.coll_results.aggregate(pipeline)
            rospy.loginfo(DURATION_OK)
            return SetBoolResponse(True, SUCCESSFUL)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

    def createUtilsResults(self):
        """Method for creating a collection of results with added task_mean_information and delta_time based on t_end-t_start

        Returns:
            bool: True if no exception occure, False viceversa
        """
        rospy.loginfo(RED + "Create Utils Results called" + END)
        # Delete existing collection if it exists
        self.coll_utils_results.delete_many({})

        # TODO FIRST CHECK THAT IN DURATION COLLECTION THERE IS SOMETHING
        # self.coll_durations.count_documents({})
        # self.coll_interaction.delete_many({})
        t_start = rospy.Time.now()

        # Pipeline = add Field delta_time and search task mean information from duration document
        pipeline = Pipeline.createUtilsResultsPipeline(self.coll_durations_name,
                                                       self.utils_results_name)

        try:
            results = self.coll_results.aggregate(pipeline)
            t_end = rospy.Time.now()

            rospy.loginfo(RED + "Computation time: " + str((t_end - t_start).to_sec()) + END)

            rospy.loginfo("Created utils results")
            return True
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return False

    def getConcurrentTaskStatistics(self, task_name: str):
        """ Method returning concurrent task informations

        Args:
            task_name (str): task name

        Returns:
            _type_: _description_
        """
        if task_name not in self.concurrent_task_counters:
            return 0, 1
        return self.concurrent_task_counters[task_name]["counter"], self.concurrent_task_counters[task_name][
                                                                        "success_counter"] / \
                                                                    self.concurrent_task_counters[task_name]["counter"]

    def getCounter(self, task_name):
        pass

    def resetConcurrentTaskCounters(self):
        self.concurrent_task_counters = dict()

    def updateConcurrentTaskCounters(self, task_name, task_outcome):
        if task_name not in self.concurrent_task_counters:
            self.concurrent_task_counters[task_name] = {"task_name": task_name, "counter": 0, "success_counter": 0}

        self.concurrent_task_counters[task_name]["counter"] += 1
        self.concurrent_task_counters[task_name]["success_counter"] += task_outcome

    def computeDynamicRisk(self, request):  # Compute dynamic risk with concurrent and mean information
        """Method for creating a collection of results with added the concurrent tasks

        Args:
            request (SetBoolRequest): Service request

        Returns:
            SetBoolResponse: Service response
        """

        t_start = rospy.Time.now()

        # Delete older dynamic risk collection
        try:
            self.coll_interaction.delete_many({})
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        # Create collection with results task + task mean information
        if not self.createUtilsResults():
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        # Pipeline for: retriving task name, unwinded for agents, each task only 1 agents
        pipeline_agents_task_name = [
            {
                '$unwind': {
                    'path': '$agent',
                    'preserveNullAndEmptyArrays': False
                }
            }, {
                '$project': {
                    '_id': 0,
                    'name': 1,
                    'agent': 1
                }
            }
        ]

        try:
            cursor_task_properties = self.coll_skills.aggregate(pipeline_agents_task_name)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        task_index = dict()  # It will be as: {"agent_name":[list with all tasks that it can perform],..}
        for single_task in cursor_task_properties:
            if "name" in single_task.keys() and "agent" in single_task.keys():  # Ensure it has nedded attributes
                if not single_task["name"] == "end" and not single_task[
                                                                "name"] == "go_home":  # Excule task end (not interesting)
                    if single_task["agent"] not in task_index:  # If not exist already that agents
                        task_index[single_task["agent"]] = set()  # A SET for each agent to ensure unique task
                    task_index[single_task["agent"]].add(single_task["name"])

        task_index = {key: list(values) for key, values in
                      task_index.items()}  # Convert agents tasks from set to list (for have index)

        rospy.loginfo("Dizionario: agente->task list: ")
        rospy.loginfo(task_index)
        rospy.loginfo(RED + "OK FINO TASK RETRIEVE INFO" + END)

        agents = list(task_index.keys())

        if len(agents) > 2:
            rospy.loginfo(RED + "There are more than 2 agents in task properties" + END)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        ############
        # Retrive results with their concurrent tasks information
        pipeline = Pipeline.dynamicRiskPipeline(self.utils_results_name)

        try:
            t_start = rospy.Time.now()
            results = self.coll_utils_results.aggregate(pipeline)
            t_end = (rospy.Time.now() - t_start).to_sec()
            rospy.loginfo("Computational time for dynamic risk" + str(t_end))
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        rospy.loginfo(YELLOW + "Iterating results" + END)

        available_stats = {"average_task_duration": "expected_duration", "min_task_duration": "min"}
        used_stat_index = available_stats["average_task_duration"]

        rospy.loginfo(GREEN + "Agents: {}".format(agents) + END)
        for task_group in results:  # A task_group contains vector of all (task_j, agent_i) same task computed by same agent
            input("New task group...")

            # Number of rows of regression matrix
            n_rows = len(task_group['grouped_task_agent'])

            # print(agents)

            # Compute other agents different from current task agent
            agent = task_group["_id"][1]  # First element of _id is name
            concurrent_agent = list(set(agents).difference(set([agent])))[
                0]  # If only one agent ok, otherwise there is also others agents in the set.

            rospy.loginfo("Principal agent: {}".format(agent))
            rospy.loginfo("Concurrent agent: {}".format(concurrent_agent))

            # for concurrent_agent in other_agents:                   #If more than one agents

            # Number of column of regression matrix = concurrent_agent task
            n_col = len(task_index[concurrent_agent])

            # Initialize regression matrix

            # regression_mat = np.zeros((n_rows,n_col))                       # To change: non è detto che per un task ce ne sia uno parallelo
            # row_vect = np.zeros((1,n_col))

            regression_mat = np.empty((0, n_col))
            known_vect = np.empty((0, 1))

            # print("////////////////////////")
            # # print(regression_mat)
            # print("////////////////////////")
            # print(n_col)
            # print(n_rows)
            # print("-----------------------")

            rospy.loginfo(GREEN + "Fondamental task: {}".format(task_group["_id"][0]) + END)

            self.resetConcurrentTaskCounters()
            # if not(agent == "ur5_on_guide" and concurrent_agent == "human_right_arm" and task_group["_id"][0]=="pick_blue_box"):
            #     continue
            for row, single_task in enumerate(task_group['grouped_task_agent']):
                add_task_to_regmat = False
                row_vect = np.zeros((1, n_col))
                # print(single_task)
                # print("task")
                # print(single_task["name"])
                # print("t start")
                # print(single_task["t_start"])

                # print("t end")
                # print(single_task["t_end"])
                # print("mean")
                # print(single_task["task_mean_informations"][0]["expected_duration"])

                p_initial_task = single_task['partial_task_initial']
                p_final_task = single_task['partial_task_final']
                # inner_tasks = single_task['inner_task']
                global_outer_task = single_task['outer_task']

                overlapping_initial_time = 0.0
                overlapping_final_time = 0.0
                overlapping_inner_tasks = 0.0
                overlapping_global_outer_task = 0.0

                # print(p_initial_task)
                # print(p_final_task)
                print("**************************************************")
                rospy.loginfo(RED + "DURATION: {}".format(single_task["delta_time"]) + END)
                # Insert partial initial task
                if p_initial_task:  # Not empty
                    if len(p_initial_task) > 1:  # Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                        rospy.loginfo(RED + "More than one partial initial task" + END)
                    else:
                        if p_initial_task[0]["outcome"] == 1:
                            # print("initial")
                            rospy.loginfo(RED + "Initial task: {}".format(p_initial_task[0]["name"]) + END)

                            overlapping_initial_time = p_initial_task[0]["t_end"] - single_task["t_start"]
                            delta_initial = overlapping_initial_time / p_initial_task[0]["delta_time"]
                            # print(p_initial_task[0])
                            # print(p_initial_task[0]["task_mean_informations"])
                            # print(p_initial_task[0]["task_mean_informations"][0])
                            # print(p_initial_task[0]["task_mean_informations"][0]["expected_duration"])
                            t_initial_mean = p_initial_task[0]["task_mean_informations"][0][
                                used_stat_index]  # "task_mean_informations":[{"exp_dur":**}]
                            # print(task_index[concurrent_agent])
                            col_index_reg_mat = task_index[concurrent_agent].index(p_initial_task[0]["name"])

                            # regression_mat[row, col_index_reg_mat] = delta_initial*t_initial_mean

                            row_vect[0, col_index_reg_mat] += delta_initial * t_initial_mean
                            add_task_to_regmat = True

                        self.updateConcurrentTaskCounters(p_initial_task[0]["name"], p_initial_task[0]["outcome"])

                        # print(single_task)
                        # print(p_initial_task)
                        # print(p_initial_task[0]["name"])
                        # print(p_initial_task[0]["agent"])
                        # print(p_initial_task[0]["t_start"])
                        # print(p_initial_task[0]["t_end"])       

                        # print(overlapping_initial_time)
                        # print(delta_initial)
                        # print(t_initial_mean)
                        # print(col_index_reg_mat)

                # Insert partial final task
                if p_final_task:  # Not empty
                    if len(p_final_task) > 1:
                        rospy.loginfo(
                            RED + "More than one partial final task" + END)  # Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                    else:
                        if p_final_task[0]["outcome"] == 1:
                            rospy.loginfo(RED + "Final task: {}".format(p_final_task[0]["name"]) + END)

                            overlapping_final_time = single_task["t_end"] - p_final_task[0]["t_start"]
                            delta_final = overlapping_final_time / p_final_task[0]["delta_time"]
                            # print(p_final_task[0]["task_mean_informations"])
                            # print(p_final_task[0]["task_mean_informations"][0])
                            # print(p_final_task[0]["task_mean_informations"][0]["expected_duration"])
                            t_final_mean = p_final_task[0]["task_mean_informations"][0][
                                used_stat_index]  # "task_mean_informations":[{"exp_dur":**}]
                            # print(concurrent_agent)
                            col_index_reg_mat = task_index[concurrent_agent].index(p_final_task[0]["name"])

                            # regression_mat[row, col_index_reg_mat] = delta_final*t_final_mean
                            row_vect[0, col_index_reg_mat] += delta_final * t_final_mean
                            add_task_to_regmat = True

                        self.updateConcurrentTaskCounters(p_final_task[0]["name"], p_final_task[0]["outcome"])
                        # print(single_task)

                        # print(p_final_task[0]["name"])
                        # print(p_final_task[0]["agent"])
                        # print(p_final_task[0]["t_start"])
                        # print(p_final_task[0]["t_end"])       

                        # print(overlapping_final_time)
                        # print(delta_final)
                        # print(t_final_mean)
                        # print(col_index_reg_mat)

                if single_task['inner_task']:
                    rospy.loginfo(RED + "Inner tasks: " + END)
                    # print(single_task['inner_task'])
                    # print("...............................................")
                    # print(single_task['inner_task'])
                    # print(type(single_task['inner_task']))
                    for inner_task in single_task['inner_task']:
                        # print(inner_task)
                        if inner_task["outcome"] == 1:  # QUesto [0] non sono sicuro
                            col_index_reg_mat = task_index[concurrent_agent].index(inner_task["name"])
                            # print("another inner tasks")
                            print(inner_task["name"])
                            # print(inner_task["task_mean_informations"])
                            # print(inner_task["task_mean_informations"][0]["expected_duration"])                        
                            # print(inner_task["task_mean_informations"][0])
                            # regression_mat[row, col_index_reg_mat] = inner_task["task_mean_informations"][0]["expected_duration"]

                            row_vect[0, col_index_reg_mat] += inner_task["task_mean_informations"][0][used_stat_index]

                            overlapping_inner_tasks += inner_task["delta_time"]
                            add_task_to_regmat = True

                        self.updateConcurrentTaskCounters(inner_task["name"], inner_task["outcome"])
                        # input("guarda inner task")
                        # print(inner_task["name"])

                if global_outer_task:
                    if len(global_outer_task) > 1:
                        rospy.loginfo(
                            RED + "More than one global outside final task" + END)  # Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                    else:
                        if global_outer_task[0]["outcome"] == 1:
                            rospy.loginfo(RED + "Global outside task: {}".format(global_outer_task[0]["name"]) + END)

                            delta_global_outside = single_task["delta_time"] / global_outer_task[0]["delta_time"]

                            t_global_outside_task_mean = global_outer_task[0]["task_mean_informations"][0][
                                used_stat_index]  # "task_mean_informations":[{"exp_dur":**}]

                            col_index_reg_mat = task_index[concurrent_agent].index(global_outer_task[0]["name"])
                            # regression_mat[row, col_index_reg_mat] = delta_global_outside * t_global_outside_task_mean     # delta_time is the real duration of "inner task"

                            row_vect[0, col_index_reg_mat] += delta_global_outside * t_global_outside_task_mean

                            overlapping_global_outer_task = single_task["delta_time"]  # All the little task   |---|
                            add_task_to_regmat = True  # |--------|
                        self.updateConcurrentTaskCounters(global_outer_task[0]["name"], global_outer_task[0]["outcome"])
                print(row_vect)
                print(overlapping_initial_time)
                print(overlapping_inner_tasks)
                print(overlapping_final_time)
                print(overlapping_global_outer_task)
                if add_task_to_regmat:
                    # Nota TODO solo se almeno uno di quelli sopra
                    regression_mat = np.append(regression_mat, row_vect, axis=0)
                    # print("------------------")
                    # print(regression_mat)
                    # print("------------------")

                    known_vect = np.append(known_vect,
                                           overlapping_initial_time + overlapping_final_time + overlapping_inner_tasks + overlapping_global_outer_task)
                    # print(known_vect)
                    # print("------------------")
                    # known_vect[row] = overlapping_initial_time + overlapping_final_time + overlapping_inner_tasks + overlapping_global_outer_task  # = single_task["delta_time"]-t_idle : t_idle = single_task["delta_time"] - overlapping_inner -overl_initial-over_final 
                # print(known_vect)
                print(known_vect)
                input("COntrolla")
            input("Regression...")
            print(regression_mat)
            print(known_vect)
            # print(regression_mat.shape)
            # print(known_vect.shape)

            t_start = rospy.Time.now()
            try:
                lstsq_results = np.linalg.lstsq(regression_mat, known_vect, rcond=None)
                print(lstsq_results)
                dynamic_risk = lstsq_results[0]
            except np.linalg.LinAlgError:
                rospy.loginfo(RED + "Least square does not converge" + END)
                return SetBoolResponse(False, NOT_SUCCESSFUL)

                # Here agent and concurrent_agent change role for dynamic risk
            for index, task in enumerate(task_index[concurrent_agent]):
                counter, success_rate = self.getConcurrentTaskStatistics(task)
                try:
                    results = self.coll_utils_results.aggregate(pipeline)
                    self.coll_interaction.insert_one({"agent": concurrent_agent,
                                                      "concurrent_agent": agent,
                                                      "agent_skill": task,
                                                      "concurrent_skill": task_group["_id"][0],
                                                      "success_rate": success_rate,
                                                      "dynamic_risk": dynamic_risk[index],
                                                      "counter": counter})
                except pymongo.errors.AutoReconnect:
                    rospy.logerr(CONNECTION_LOST)
                    return SetBoolResponse(False, NOT_SUCCESSFUL)

            rospy.loginfo(GREEN + "Fondamental task:" + END)
            print(single_task["name"])
            print(task_index[concurrent_agent])
            print(dynamic_risk)
            input("Leggi i risultati...")
            # for k in coefficient:
            #     input("Prossimo k")
            #     print(k)

            print((rospy.Time.now() - t_start).to_sec())

        return SetBoolResponse(True, SUCCESSFUL)

    def dynamicRiskChart(self, request):
        """Method that is a callback of a service usefull for do dynamic risk heat map

        Args:
            request (SetBoolRequest): _description_

        Returns:
            SetBoolResponse: _description_
        """

        # Check if the results folder path not exist create it
        if not os.path.exists(self.results_folder_path):
            os.makedirs(self.results_folder_path)
            rospy.loginfo(RED + "The new direcory {} is created!".format(self.results_folder_path) + END)

        pipeline_dr_grouped_by_agent = Pipeline.groupedDynamicRiskPipeline()  # Pipeline to group/separate dynamic risk elements by agent

        try:
            dynamic_risk_grouped = self.coll_interaction.aggregate(pipeline_dr_grouped_by_agent)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)

        dynamic_risk_list = []  # It will be a list of dictionaries each made like: {"main_agent":"name", "concurrent_agent":"name", "name_main_agent":[],  "name_concurrent_agent":[], "dynamic_risk":[]}

        for index, dynamic_risk_single_agent in enumerate(
                dynamic_risk_grouped):  # Iterate a group of dynamic_risk element (a group for main agent)
            dynamic_risk_list.append({})
            dynamic_risk_list[index][dynamic_risk_single_agent["_id"]] = []  # Main agent
            dynamic_risk_list[index]["dynamic_risk"] = []
            dynamic_risk_list[index]["main_agent"] = dynamic_risk_single_agent[
                "_id"]  # In order to store who is main agent
            for dynamic_risk_single_element in dynamic_risk_single_agent["grouped_task_agent"]:
                # print(dynamic_risk_single_element)

                # Add concurrent agent  
                if dynamic_risk_single_element["concurrent_agent"] not in dynamic_risk_list[index]:
                    dynamic_risk_list[index][dynamic_risk_single_element["concurrent_agent"]] = []
                    dynamic_risk_list[index]["concurrent_agent"] = dynamic_risk_single_element[
                        "concurrent_agent"]  # In order to store who is concurrent agent
                    # here dynamic_risk_list[index] = {"main_agent":"name", "concurrent_agent":"name", "name_main_agent":[],  "name_concurrent_agent":[], "dynamic_risk":[]}

                # Retrieve agent and concurrent agent skills 
                agent_skill = dynamic_risk_single_element["agent_skill"]
                concurrent_agent_skill = dynamic_risk_single_element["concurrent_skill"]

                # If pick_blue_box -> pick_blue_box_ + agent
                if "pick_blue_box" in dynamic_risk_single_element["agent_skill"]:
                    agent_skill += "_" + dynamic_risk_single_element["agent"]
                if "pick_blue_box" in dynamic_risk_single_element["concurrent_skill"]:
                    concurrent_agent_skill += "_" + dynamic_risk_single_element["concurrent_agent"]

                # Append dynamic_risk element 
                dynamic_risk_list[index][dynamic_risk_single_agent["_id"]].append(agent_skill)
                dynamic_risk_list[index][dynamic_risk_single_element["concurrent_agent"]].append(concurrent_agent_skill)
                dynamic_risk_list[index]["dynamic_risk"].append(dynamic_risk_single_element["dynamic_risk"])
            print("-------------")
            print("Main agent: {}".format(dynamic_risk_single_agent["_id"]))
            print("Concurrent agent: {}".format(dynamic_risk_single_element["concurrent_agent"]))
            print(dynamic_risk_list[index])

            # fig, axs = plt.subplots(1, 2)
        # fig.suptitle('Dynamic Matrix')

        for index, single_agent_dynamic_risk in enumerate(dynamic_risk_list):
            # Retrieve agent name
            main_agent = single_agent_dynamic_risk["main_agent"]
            concurrent_agent = single_agent_dynamic_risk["concurrent_agent"]

            # Remove it for chart data
            single_agent_dynamic_risk.pop("main_agent", None)
            single_agent_dynamic_risk.pop("concurrent_agent", None)

            # Create a dataframe
            data = pd.DataFrame(single_agent_dynamic_risk)
            print(data)
            print(True in data.duplicated())
            if data.duplicated().any():
                rospy.loginfo(RED + "There are duplicated task" + END)
                rospy.loginfo(data.duplicated())
                return SetBoolResponse(False, NOT_SUCCESSFUL)
            print(main_agent)
            print(concurrent_agent)
            # Prepare data for heatmap
            # data_matrix = data.pivot(main_agent, concurrent_agent, "dynamic_risk")
            data_matrix = data.pivot(index=main_agent, columns=concurrent_agent, values='dynamic_risk')

            print(data)
            print("qui")
            plot_title = "Sinergy Matrix for agent: {}".format(main_agent)
            # TODO: Fix paper names
            paper = True
            if paper:
                if main_agent == "ur5_on_guide" or main_agent == "manipulator":
                    main_agent_label = {"name": "Robot", "abbreviation": "R"}  # ("Robot","R")
                    concurrent_agent_label = {"name": "Human", "abbreviation": "H"}  # ("Robot","R")

                elif main_agent == "human_right_arm" or main_agent == "human":
                    main_agent_label = {"name": "Human", "abbreviation": "H"}  # ("Human","H")
                    concurrent_agent_label = {"name": "Robot", "abbreviation": "R"}
                else:
                    main_agent_label = {"name": main_agent, "abbreviation": main_agent}
                plot_title = "Sinergy Matrix for agent: {} ($S^{}$)".format(main_agent_label["name"],
                                                                            main_agent_label["abbreviation"])
                for agent in [main_agent, concurrent_agent]:
                    for k in data.index:
                        if paper:
                            data.at[k, agent] = self.getPaperTaskName(data.at[k, agent])
                data = data.rename(columns={main_agent: main_agent_label["name"] + " Tasks",
                                            concurrent_agent: concurrent_agent_label["name"] + " Tasks"})
                print(data)
                print(main_agent_label["name"] + " Tasks")
                print(concurrent_agent_label["name"] + " Tasks")
                data_matrix = data.pivot(index=main_agent_label["name"] + " Tasks",
                                         columns=concurrent_agent_label["name"] + " Tasks",
                                         values="dynamic_risk")

            else:
                main_agent_label = main_agent  # Remove label variable (unuseful)

            rospy.loginfo(RED + " ----------- " + END)

            print(data_matrix)
            rospy.loginfo(RED + " ----------- " + END)
            print(data_matrix)
            sns.set_theme()

            def plot_a_graph():
                # print("CHARTTT")
                plt.figure(index)
                # sns.set_theme()
                ax = sns.heatmap(data_matrix, annot=True, cmap="flare")
                # plt.title(plot_title)
                # ax.set_title(plot_title, pad=20)
                plt.ylabel("Robot Tasks", labelpad=25)
                # TODO: fix results folder path
                plt.xticks(rotation=20, ha="right")
                self.results_folder_path = "/home/galois/Documents/"
                plt.savefig(self.results_folder_path + "dynamic_risk_agent_" + main_agent + ".png",
                            bbox_inches='tight')
                plt.savefig(self.results_folder_path + "test" + "dynamic_risk_agent_" + main_agent + ".pdf",
                            bbox_inches='tight')

                plt.show()
                print(multiprocessing.current_process().name)

                # Plotly

                # import plotly.tools as tls
                # import plotly.graph_objects as go
                # def df_to_plotly(df):
                #     return {'z': df.values.tolist(),
                #             'x': df.columns.tolist(),
                #             'y': df.index.tolist()}
                #
                # fig = go.Figure(data=go.Heatmap(df_to_plotly(data_matrix),text=data_matrix.values.tolist(), hoverongaps=False, colorscale='Viridis', showscale=False))
                # # Aggiungi le annotazioni
                # for i in range(len(data_matrix.index)):
                #     for j in range(len(data_matrix.columns)):
                #         fig.add_annotation(
                #             x=data_matrix.columns[j],
                #             y=data_matrix.index[i],
                #             text=str(data_matrix.values[i][j]),
                #             showarrow=False,
                #             font=dict(color="white" if data_matrix.values[i][j] < 0.5 else "black")
                #         )
                # fig.show()

            job_for_another_core = multiprocessing.Process(target=plot_a_graph, args=())
            job_for_another_core.start()

        return SetBoolResponse(True, SUCCESSFUL)

        t_start = rospy.Time.now()

        rospy.loginfo(SERVICE_CALLBACK.format(COMPUTE_DURATION_SERVICE))
        # Aggiunge delta_time, aggiunge la duration media del task "principale", + concurrent che sono senza info medie
        pipeline = Pipeline.dynamicRiskNoAddInfo(self.coll_durations_name,
                                                 self.coll_results_name)

        try:
            results = self.coll_results.aggregate(pipeline)
            t_end = rospy.Time.now()
            for single_task in results:
                if "name" in single_task:
                    print("Principal task:")
                    print(single_task['name'])
                for concurrent_task_final_single in single_task['partial_task_final']:
                    print("Concurrent task final:")
                    print(concurrent_task_final_single)
                    print("---------------------------------------------------------\n")

                for concurrent_task_initial_single in single_task['partial_task_initial']:
                    print("Concurrent task initial:")
                    print(concurrent_task_initial_single)
                    print("---------------------------------------------------------\n")

                for concurrent_task_inside_single in single_task['inner_task']:
                    print("Concurrent task inside:")
                    print(concurrent_task_inside_single)
                    print("---------------------------------------------------------\n")
                print("---------------------------------------------------------\n")
            rospy.loginfo(RED + "Tempo impiegato: " + str((t_end - t_start).to_sec()) + END)

            # pprint.pprint(list(results))
            # rospy.loginfo(DURATION_OK)
            return SetBoolResponse(True, SUCCESSFUL)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

    def drawTimeline(self, request):
        # Function to optimize for more than one agents and better graphics skills
        pipeline = [
            {
                '$group': {
                    '_id': '$recipe',
                    'recipe_tasks': {
                        '$push': '$$ROOT'
                    }
                }
            }
        ]

        try:
            results = self.coll_utils_results.aggregate(pipeline)
            n_recipe = 0
            for task in results:
                print("Single task in recipe:")
                # print(task)
                recipe_task = []

                recipe_robot_task = []
                recipe_human_task = []
                recipe_robot_task_name = []
                recipe_human_task_name = []
                rospy.loginfo(RED + "--------------------------------------------" + END)
                if "recipe_tasks" in task.keys():
                    fig, ax = plt.subplots()
                    for single_task in task["recipe_tasks"]:
                        # print(single_task)
                        # print((single_task['t_start'], single_task['delta_time'], single_task['agent']))
                        single_task['t_start']
                        if single_task['agent'] == "ur5_on_guide":
                            recipe_robot_task.append((single_task['t_start'], single_task['delta_time']))
                            recipe_robot_task_name.append(single_task["name"])
                        else:
                            recipe_human_task.append((single_task['t_start'], single_task['delta_time']))
                            recipe_human_task_name.append(single_task["name"])
                        recipe_task.append((single_task['t_start'], single_task['delta_time'], single_task['agent']))
                    print(recipe_robot_task)
                    print(recipe_human_task)
                    # recipe_robot_task = list(filter(lambda single_task: single_task[2]=="motion", recipe_task))
                    # recipe_human_task = list(filter(lambda single_task: single_task[2]=="human", recipe_task))

                    # Setting Y-axis limits
                    recipe_robot_task.sort(key=lambda single_task: single_task[0])
                    recipe_human_task.sort(key=lambda single_task: single_task[0])
                    recipe_task.sort(key=lambda single_task: single_task[0])

                    if recipe_task:
                        print(recipe_task)
                        rospy.loginfo(RED + "T start" + END)
                        t_start_recipe = recipe_task[0][0]
                        print(t_start_recipe)
                        recipe_task = list(
                            map(lambda single_task: (single_task[0] - t_start_recipe, single_task[1]), recipe_task))
                        t_end_recipe = recipe_task[-1][0] + recipe_task[-1][1]
                        rospy.loginfo(RED + "T END" + END)
                        rospy.loginfo(t_end_recipe)
                        # rospy.loginfo(recipe_task)
                        rospy.loginfo(t_end_recipe)
                        recipe_robot_task = list(
                            map(lambda single_task: (single_task[0] - t_start_recipe, single_task[1]),
                                recipe_robot_task))
                        recipe_human_task = list(
                            map(lambda single_task: (single_task[0] - t_start_recipe, single_task[1]),
                                recipe_human_task))

                    rospy.loginfo(RED + "-----------------" + END)
                    # print(recipe_robot_task_name)
                    # print(recipe_human_task_name)
                    color_robot_task = []
                    color_human_task = []
                    for robot_task_name in recipe_robot_task_name:
                        if "orange" in robot_task_name:
                            if "pick" in robot_task_name:
                                color_robot_task.append([0.9, 0.4, 0.1, 1])
                            else:
                                color_robot_task.append([0.9, 0.5, 0.0, 0.8])
                        elif "blue" in robot_task_name:
                            if "pick" in robot_task_name:
                                color_robot_task.append([0, 0.4, 0.6, 1])
                            else:
                                color_robot_task.append([0, 0.6, 0.9, 0.8])
                        elif "white" in robot_task_name:
                            if "pick" in robot_task_name:
                                color_robot_task.append([0.7, 0.7, 0.7, 1])
                            else:
                                color_robot_task.append([0.9, 0.9, 0.9, 0.8])
                    for human_task_name in color_human_task:
                        if "orange" in human_task_name:
                            if "pick" in human_task_name:
                                color_human_task.append([0.9, 0.4, 0.1, 1])
                            else:
                                color_human_task.append([0.9, 0.5, 0.0, 0.8])
                        elif "blue" in human_task_name:
                            if "pick" in robot_task_name:
                                color_human_task.append([0, 0.4, 0.6, 1])
                            else:
                                color_human_task.append([0, 0.6, 0.9, 0.8])
                        elif "white" in human_task_name:
                            if "pick" in robot_task_name:
                                color_human_task.append([0.2, 0.7, 0.2, 1])
                            else:
                                color_human_task.append([0.9, 0.7, 0.2, 0.8])
                    colors_robot_task = np.array(color_robot_task)
                    colors_human_task = np.array(color_human_task)
                    print(colors_robot_task)
                    print(colors_human_task)
                    rospy.loginfo(RED + "-----------------" + END)
                    # print("robot task: ")
                    # print(recipe_robot_task)
                    # print("human task: ")
                    # print(recipe_human_task)

                    # Setting labels for x-axis and y-axis
                    ax.set_xlabel('Time (s)')
                    ax.set_ylabel('Agents')

                    # Setting ticks on y-axis
                    # Labelling tickes of y-axis
                    ax.set_yticks([1, 4])
                    ax.set_yticklabels(['Robot', 'Human'])

                    # Setting X-axis limits
                    ax.set_xlim(0, t_end_recipe * 1.2)
                    # Setting graph attribute
                    ax.grid(True)

                    # colors_robot_task = plt.cm.BuPu(np.linspace(0, 0.5, len(recipe_robot_task)))
                    # colors_human_task = plt.cm.BuPu(np.linspace(0, 0.5, len(recipe_human_task)))

                    print(colors_robot_task)

                    # mcolors.BASE_COLORS #these colors can be called with a single character

                    # mcolors.TABLEAU_COLORS #the default color cycle colors

                    # mcolors.CSS4_COLORS #named colors also recognized in css

                    # mcolors.XKCD_COLORS #named colors from the xkcd survey

                    if recipe_robot_task:
                        print("Robot task")
                        print(recipe_robot_task)
                        print(len(recipe_robot_task))
                        ax.broken_barh(recipe_robot_task,
                                       (0, 2),
                                       facecolors=colors_robot_task)
                    if recipe_human_task:
                        print("Human task")
                        print(recipe_human_task)
                        print(len(recipe_human_task))
                        ax.broken_barh(recipe_human_task,
                                       (3, 2),
                                       facecolors=colors_human_task)

                    ax.grid(True)
                    ax.set_title("Recipe: {}".format(n_recipe))
                    # self.results_folder_path="/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_statistics/file/recipe/"

                    if not os.path.exists(self.results_folder_path):
                        os.makedirs(self.results_folder_path)
                        rospy.loginfo(RED + "The new direcory {} is created!".format(self.results_folder_path) + END)
                    plt.savefig(self.results_folder_path + "recipe" + str(n_recipe) + ".png")
                    rospy.loginfo(RED + "Recipe: {}".format(n_recipe) + END)

                    # print(recipe_task)
                    # print("single task")
                    # print(task['recipe_tasks'])
                    input("Wait ..")
                    n_recipe += 1

            rospy.loginfo("Chart Results ok")
            return SetBoolResponse(True, SUCCESSFUL)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        pass

    def groupedDurationChart(self, request):  # Compute dynamic risk with concurrent and mean information
        """Method for making chart of task duration (grouped task for each parallel task)

        Args:
            request (SetBoolRequest): 

        Returns:
            SetBoolResponse: 
        """

        if not self.createUtilsResults():
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        agents = self.getAgents()

        task_results = dict()
        task_results["duration"] = []
        for agent in agents:
            task_results[agent] = []

        pipeline = Pipeline.dynamicRiskForChart(self.utils_results_name)

        try:
            results = self.coll_utils_results.aggregate(pipeline)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        rospy.loginfo(YELLOW + "Iterating results" + END)

        rospy.loginfo(GREEN + "Agents: {}".format(agents) + END)
        for main_agent_group in results:
            main_agent = main_agent_group["_id"]
            concurrent_agent = list(set(agents).difference(set([main_agent])))[
                0]  # If only one agent ok, otherwise there is also others agents in the set.

            task_results_agent = copy.deepcopy(task_results)
            print(task_results_agent)
            input("verifica...")

            for task_group in main_agent_group[
                "grouped_main_agent"]:  # A task_group contains vector of all (task_j, agent_i) same task computed by same agent
                # input("New task group...")

                # Number of rows of regression matrix

                # Compute other agents different from current task agent
                agent = task_group["_id"][1]  # First element of _id is name

                rospy.loginfo("Principal agent: {}".format(agent))
                rospy.loginfo("Concurrent agent: {}".format(concurrent_agent))

                rospy.loginfo(GREEN + "Fondamental task: {}".format(task_group["_id"][0]) + END)

                # task_results = dict()
                for single_task in task_group['grouped_task_agent']:

                    p_initial_task = single_task['partial_task_initial']
                    p_final_task = single_task['partial_task_final']
                    global_outer_task = single_task['outer_task']

                    rospy.loginfo(RED + "DURATION: {}".format(single_task["delta_time"]) + END)
                    # Insert partial initial task
                    if p_initial_task:  # Not empty
                        if len(p_initial_task) > 1:  # Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                            rospy.loginfo(RED + "More than one partial initial task" + END)
                        else:
                            if p_initial_task[0]["outcome"] == 1:
                                rospy.loginfo(RED + "Initial task: {}".format(p_initial_task[0]["name"]) + END)
                                print(task_results)
                                task_results_agent[agent].append(single_task["name"])
                                task_results_agent[concurrent_agent].append(p_initial_task[0]["name"])
                                task_results_agent["duration"].append(single_task["delta_time"])
                    # Insert partial final task
                    if p_final_task:  # Not empty
                        if len(p_final_task) > 1:
                            rospy.loginfo(
                                RED + "More than one partial final task" + END)  # Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                        else:
                            if p_final_task[0]["outcome"] == 1:
                                rospy.loginfo(RED + "Final task: {}".format(p_final_task[0]["name"]) + END)
                                task_results_agent[agent].append(single_task["name"])
                                task_results_agent[concurrent_agent].append(p_final_task[0]["name"])
                                task_results_agent["duration"].append(single_task["delta_time"])

                    if single_task['inner_task']:
                        rospy.loginfo(RED + "Inner tasks: " + END)
                        for inner_task in single_task['inner_task']:
                            # print(inner_task)
                            if inner_task["outcome"] == 1:  # QUesto [0] non sono sicuro
                                print(inner_task["name"])

                                task_results_agent[agent].append(single_task["name"])
                                task_results_agent[concurrent_agent].append(inner_task["name"])
                                task_results_agent["duration"].append(single_task["delta_time"])

                    if global_outer_task:
                        if len(global_outer_task) > 1:
                            rospy.loginfo(
                                RED + "More than one global outside final task" + END)  # Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                        else:
                            if global_outer_task[0]["outcome"] == 1:
                                rospy.loginfo(
                                    RED + "Global outside task: {}".format(global_outer_task[0]["name"]) + END)
                                task_results_agent[agent].append(single_task["name"])
                                task_results_agent[concurrent_agent].append(global_outer_task[0]["name"])
                                task_results_agent["duration"].append(single_task["delta_time"])

            print(task_results_agent)

            dati = pd.DataFrame(task_results_agent)
            print(dati)

            # sns.set_theme()
            # # sns.catplot(data=dati, x=main_agent, y="duration", hue=concurrent_agent)
            # sns.catplot(data=dati, kind="bar", x=main_agent, y="duration", hue=concurrent_agent)

            # plt.figure()
            def plot_a_graph():
                # plt.figure()

                sns.set_theme()
                # sns.catplot(data=dati, x=main_agent, y="duration", hue=concurrent_agent)
                sns.catplot(data=dati, kind="bar", x=main_agent, y="duration", hue=concurrent_agent)
                plt.show()

            job_for_another_core = multiprocessing.Process(target=plot_a_graph, args=())
            job_for_another_core.start()

        # plt.show()

        return SetBoolResponse(True, SUCCESSFUL)

    def getAgents(self):
        """Utility method that group all agents in task_properties db and return it in a list

        Raises:
            pymongo.errors.AutoReconnect: If the connection is lost

        Returns:
            _type_: A list containing all agents name 
        """
        pipeline = [
            {
                '$unwind': {
                    'path': '$agent',
                    'preserveNullAndEmptyArrays': False
                }
            }, {
                '$group': {
                    '_id': '$agent'
                }
            }
        ]
        try:
            agents_cursor = self.coll_utils_results.aggregate(pipeline)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            raise pymongo.errors.AutoReconnect

        return [agent["_id"] for agent in list(agents_cursor)]

    def computeOverlappingRatio(self, parallelism_task_type, main_task_start, main_task_end, parallel_task_start,
                                parallel_task_end):

        if parallelism_task_type == PARTIAL_INITIAL:
            overlapping_ratio = parallel_task_end - main_task_start
        elif parallelism_task_type == PARTIAL_FINAL:
            overlapping_ratio = main_task_end - parallel_task_start
        elif parallelism_task_type == INNER:
            overlapping_ratio = parallel_task_end - parallel_task_start
        elif parallelism_task_type == OUTER:
            overlapping_ratio = main_task_end - main_task_start
        else:
            overlapping_ratio = None

        assert overlapping_ratio > 0

        return overlapping_ratio

        # TODO: Receive in input task if initial, ..., and time start-end of main task and concurrent task and parallel task

    def computeDynamicRiskWithUncertainty(self, request):  # Compute dynamic risk with concurrent and mean information
        """Method for creating a collection of results with added the concurrent tasks

        Args:
            request (SetBoolRequest): _description_

        Returns:
            SetBoolResponse: _description_
        """

        t_start = rospy.Time.now()

        # Delete older dynamic risk collection
        try:
            self.coll_interaction.delete_many({})
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        available_stats = {"average_task_duration": "expected_duration", "min_task_duration": "min",
                           "median_task_duration": "median"}

        used_stat_index = available_stats["average_task_duration"]
        if used_stat_index == "median":
            try:
                self.addMedianToTaskStats()
            except pymongo.errors.AutoReconnect:
                rospy.logerr(CONNECTION_LOST)
                return SetBoolResponse(False, NOT_SUCCESSFUL)

        # Create collection with results task + task mean information
        if not self.createUtilsResults():
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        # Pipeline for: retriving task name, unwinded for agents, each task only 1 agents
        pipeline_agents_task_name = [
            {
                '$unwind': {
                    'path': '$agent',
                    'preserveNullAndEmptyArrays': False
                }
            }, {
                '$project': {
                    '_id': 0,
                    'name': 1,
                    'agent': 1
                }
            }
        ]

        try:
            cursor_task_properties = self.coll_skills.aggregate(pipeline_agents_task_name)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        task_index = dict()  # It will be as: {"agent_name":[list with all tasks that it can perform],..}
        for single_task in cursor_task_properties:
            if "name" in single_task.keys() and "agent" in single_task.keys():  # Ensure it has nedded attributes
                if not single_task["name"] == "end" and not single_task[
                                                                "name"] == "go_home":  # Excule task end (not interesting)
                    if single_task["agent"] not in task_index:  # If not exist already that agents
                        task_index[single_task["agent"]] = set()  # A SET for each agent to ensure unique task
                    task_index[single_task["agent"]].add(single_task["name"])

        task_index = {key: list(values) for key, values in
                      task_index.items()}  # Convert agents tasks from set to list (for have index)

        rospy.loginfo("Dizionario: agente->task list: ")
        rospy.loginfo(task_index)

        # input("wait...")

        agents = list(task_index.keys())

        if len(agents) > 2:
            rospy.loginfo(RED + "There are more than 2 agents in task properties" + END)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        # Retrive results with their concurrent tasks information
        pipeline = Pipeline.dynamicRiskPipeline(self.utils_results_name)

        parallelism_type = [PARTIAL_INITIAL, PARTIAL_FINAL, INNER, OUTER]

        try:
            t_start = rospy.Time.now()
            results = self.coll_utils_results.aggregate(pipeline)
            t_end = (rospy.Time.now() - t_start).to_sec()
            rospy.loginfo("Computational time for dynamic risk" + str(t_end))
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        rospy.loginfo(GREEN + "Agents: {}".format(agents) + END)

        for task_group in results:  # A task_group contains vector of all (task_j, agent_i) same task computed by same agent
            # input("New task group...")    #usefull for debug

            # Number of rows of regression matrix
            n_rows = len(task_group['grouped_task_agent'])

            # print(agents)

            # Compute other agents different from current task agent
            agent = task_group["_id"][1]  # First element of _id is name
            concurrent_agent = set(agents).difference(
                set([agent])).pop()  # If only one agent ok, otherwise there is also others agents in the set.

            rospy.loginfo("Principal agent: {}".format(agent))
            rospy.loginfo("Concurrent agent: {}".format(concurrent_agent))

            # for concurrent_agent in other_agents:                   #If more than one agents

            # Number of column of regression matrix = concurrent_agent task
            n_col = len(task_index[concurrent_agent])

            # Initialize regression matrix

            regression_mat = np.empty((0, n_col))
            known_vect = np.empty((0, 1))

            rospy.loginfo(GREEN + "Fondamental task: {}".format(task_group["_id"][0]) + END)

            self.resetConcurrentTaskCounters()

            for row, single_task in enumerate(task_group['grouped_task_agent']):
                add_task_to_regmat = False
                row_vect = np.zeros((1, n_col))

                p_initial_task = single_task['partial_task_initial']
                p_final_task = single_task['partial_task_final']
                # inner_tasks = single_task['inner_task']
                global_outer_task = single_task['outer_task']

                overlapping_tot = 0.0
                for parallelism_task_type in parallelism_type:
                    task = single_task[parallelism_task_type]
                    if parallelism_task_type != INNER and len(task) > 1:
                        rospy.loginfo(RED + "More than one {} task".format(parallelism_task_type) + END)
                        raise Exception("More than one {} task".format(parallelism_task_type))
                    if task:
                        for parallel_task in task:  # per i task inner possono essere più di uno
                            if parallel_task["outcome"] == 1:
                                overlapping_time = self.computeOverlappingRatio(parallelism_task_type,
                                                                                single_task["t_start"],
                                                                                single_task["t_end"],
                                                                                parallel_task["t_start"],
                                                                                parallel_task["t_end"])
                                delta = overlapping_time / parallel_task["delta_time"]
                                parallel_task_stat_cost = parallel_task["task_mean_informations"][0][used_stat_index]

                                col_index_reg_mat = task_index[concurrent_agent].index(parallel_task["name"])
                                row_vect[0, col_index_reg_mat] += delta * parallel_task_stat_cost
                                overlapping_tot += overlapping_time

                                add_task_to_regmat = True
                            self.updateConcurrentTaskCounters(parallel_task["name"], parallel_task["outcome"])

                if add_task_to_regmat:
                    # Nota TODO solo se almeno uno di quelli sopra
                    regression_mat = np.append(regression_mat, row_vect, axis=0)

                    known_vect = np.append(known_vect, overlapping_tot)

            print(regression_mat)
            print(known_vect)
            rospy.loginfo(YELLOW + "****************" + END)
            input("Verifica")
            # if agent == "human_right_arm":

            clf = IsolationForest(n_estimators=20, warm_start=True)
            estimator = clf.fit(known_vect.reshape(-1, 1))  # fit 10 trees
            check = estimator.decision_function(known_vect.reshape(-1, 1))
            outliers = check > 0

            # outliers=np.abs(known_vect-np.mean(known_vect))<np.std(known_vect)*1.8
            # outliers = known_vect<15
            print(outliers)
            regression_mat = regression_mat[outliers, :]
            known_vect = known_vect[outliers]

            # print(outliers)
            rospy.loginfo(YELLOW + "****************" + END)

            print(regression_mat.shape)
            print(known_vect.shape)

            t_start = rospy.Time.now()
            try:
                lstsq_results = np.linalg.lstsq(regression_mat, known_vect, rcond=None)
                print(lstsq_results)
                dynamic_risk = lstsq_results[0]
            except np.linalg.LinAlgError:
                rospy.loginfo(RED + "Least square does not converge" + END)
                return SetBoolResponse(False, NOT_SUCCESSFUL)

            print("**************")
            (row, col) = regression_mat.shape
            dizionario = dict()
            formula_string = ""
            for index in range(0, col):
                dizionario[task_index[concurrent_agent][index]] = regression_mat[:, index]
                formula_string += task_index[concurrent_agent][index]
                if index < col - 1:
                    formula_string += " + "
            rospy.loginfo(formula_string)
            # input("Look at formula")  #usefull for debug   
            dizionario["duration"] = known_vect
            print(dizionario)
            dataF = pd.DataFrame(dizionario)

            print(dataF)
            input("Look at dataframe")

            reg_model = smf.ols(formula="duration ~ " + formula_string + " -1 ", data=dataF)
            reg_result = reg_model.fit()
            print(reg_result.summary())

            print("**************")

            # Here agent and concurrent_agent change role for dynamic risk
            for index, task in enumerate(task_index[concurrent_agent]):
                counter, success_rate = self.getConcurrentTaskStatistics(task)
                print(task)
                print(reg_result.bse[task])
                try:
                    # results = self.coll_utils_results.aggregate(pipeline)
                    self.coll_interaction.insert_one({"agent": concurrent_agent,
                                                      "concurrent_agent": agent,
                                                      "agent_skill": task,
                                                      "concurrent_skill": task_group["_id"][0],
                                                      "success_rate": success_rate,
                                                      "dynamic_risk": dynamic_risk[index],
                                                      "std_err": reg_result.bse[task],
                                                      "counter": counter})
                except pymongo.errors.AutoReconnect:
                    rospy.logerr(CONNECTION_LOST)
                    return SetBoolResponse(False, NOT_SUCCESSFUL)

            rospy.loginfo(GREEN + "Fondamental task:" + END)
            print(single_task["name"])
            print(task_index[concurrent_agent])
            print(dynamic_risk)
            # input("Look at results")

            print((rospy.Time.now() - t_start).to_sec())

        return SetBoolResponse(True, SUCCESSFUL)

    def addMedianToTaskStats(self):
        """_summary_

        Raises:
            pymongo.errors.AutoReconnect: In case of connection lost with db
        """
        pipeline_for_median = [
            {
                '$group': {
                    '_id': {
                        'agent': '$agent',
                        'name': '$name'
                    },
                    'all': {
                        '$push': '$$ROOT'
                    }
                }
            }
        ]
        try:
            cursor_task_median = self.coll_results.aggregate(pipeline_for_median)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            raise pymongo.errors.AutoReconnect

        for task_group in cursor_task_median:
            agent = task_group["_id"]["agent"]
            task_name = task_group["_id"]["name"]

            task_duration_vector = np.empty((len(task_group["all"])))

            for id, task in enumerate(task_group["all"]):
                task_duration_vector[id] = task["t_end"] - task["t_start"]
            median = np.median(task_duration_vector)

            try:
                self.coll_durations.update_one({"_id.agent": agent, "_id.name": task_name},
                                               {"$set": {"median": median}})
            except pymongo.errors.AutoReconnect:
                rospy.logerr(CONNECTION_LOST)
                raise pymongo.errors.AutoReconnect

    def singleDRCoefficientChart(self, request):
        """Method 

        Args:
            request (SetBoolRequest): 

        Returns:
            SetBoolResponse: 
        """

        agents = self.getAgents()

        print(agents)

        n_samples = 1000
        for main_agent in agents:
            synergy_agent_values = pd.DataFrame(
                columns=['main_agent', 'main_task', 'concurrent_agent', 'concurrent_task', 'synergy'], dtype=object)
            pipeline_get_only_speciefied_agent_synergy_elements = [
                {
                    '$match': {
                        'agent': main_agent
                    }
                }
            ]
            try:
                results = self.coll_interaction.aggregate(pipeline_get_only_speciefied_agent_synergy_elements)
            except pymongo.errors.AutoReconnect:
                rospy.logerr(CONNECTION_LOST)
                return SetBoolResponse(False, NOT_SUCCESSFUL)

            for agent_synergy_element in results:
                concurrent_agent = agent_synergy_element["concurrent_agent"]
                main_task = agent_synergy_element["agent_skill"]
                concurrent_task = agent_synergy_element["concurrent_skill"]
                main_agent_label = main_agent

                if paper:
                    main_task = self.getPaperTaskName(
                        self.checkPickPlaceWithAgent(agent_synergy_element["agent_skill"], main_agent))
                    concurrent_agent = self.getPaperAgentName(agent_synergy_element["concurrent_agent"])
                    concurrent_task = self.getPaperTaskName(
                        self.checkPickPlaceWithAgent(agent_synergy_element["concurrent_skill"],
                                                     agent_synergy_element["concurrent_agent"]))
                    main_agent_label = self.getPaperAgentName(main_agent)

                synergy_mean_value = agent_synergy_element["dynamic_risk"]
                synergy_std = agent_synergy_element["std_err"]
                dati = np.random.normal(synergy_mean_value, synergy_std, n_samples)

                extended_synergy = pd.DataFrame(
                    {'main_agent': main_agent, "main_task": main_task, "concurrent_agent": concurrent_agent,
                     "concurrent_task": concurrent_task, "synergy": dati}, dtype=object)

                synergy_agent_values = synergy_agent_values.append(extended_synergy, ignore_index=True)

            sns.set_theme()
            sns.set(rc={'figure.figsize': (11, 7)})
            # sns.catplot(data=dati, x=main_agent, y="duration", hue=concurrent_agent)

            ax = sns.catplot(data=synergy_agent_values, kind="bar", x="main_task", y="synergy", hue="concurrent_task",
                             ci="sd", legend=False)
            ax.set_xlabels('Main Agent: ' + main_agent_label, fontsize=20, labelpad=15)  # not set_label
            ax.fig.suptitle("Synergy Matrix Coefficients", y=1)
            ax.set_ylabels("Synergy coefficient value", fontsize=20, labelpad=15)
            plt.xticks(fontsize=16)
            plt.yticks(fontsize=16)

            legend = plt.legend(title="Concurrent task ({})".format(concurrent_agent), loc='upper left', fontsize='18')
            legend.get_title().set_fontsize('20')

            # plt.xticks(rotation=, ha="right")
            # self.results_folder_path="/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_statistics/file/"
            ax.figure.set_size_inches(11, 8)

            plt.savefig(self.results_folder_path + "dynamic_risk_coefficient_agent_" + main_agent + ".png")
            plt.savefig(self.results_folder_path + "test" + "dynamic_coefficient_risk_agent_" + ".pdf")

        # plt.figure()
        plt.show()

        return SetBoolResponse(True, "ok")

    def taskDurationChart(self, request):
        """Method for making task duration grouped by type

        Args:
            request (SetBoolRequest): Request

        Returns:
            SetBoolResponse: Response
        """

        agents = self.getAgents()
        print(agents)
        task_results = {"task_name": [], "agent_name": [], "duration": []}
        for main_agent in agents:
            pipeline_get_only_speciefied_agent_elements = [
                {
                    '$match': {
                        'agent': main_agent
                    }
                }
            ]
            try:
                results = self.coll_results.aggregate(pipeline_get_only_speciefied_agent_elements)
            except pymongo.errors.AutoReconnect:
                rospy.logerr(CONNECTION_LOST)
                return SetBoolResponse(False, NOT_SUCCESSFUL)
            for result_element in results:
                task_name = result_element["name"]
                agent_name = result_element["agent"]
                duration = result_element["t_end"] - result_element["t_start"]

                if paper:
                    task_name = self.getPaperTaskName(self.checkPickPlaceWithAgent(task_name, agent_name))
                    agent_name = self.getPaperAgentName(agent_name)

                task_results["task_name"].append(task_name)
                task_results["agent_name"].append(agent_name)
                task_results["duration"].append(duration)

        task_results_data = pd.DataFrame(task_results)

        def plot_a_graph():
            sns.set_theme()
            sns.set(rc={'figure.figsize': (17, 7)})

            # ax = sns.violinplot(data=task_results_data, x="task_name", y="duration", hue="agent_name")
            print(task_results_data)
            ax = sns.boxplot(data=task_results_data, x="task_name", y="duration", hue="agent_name",
                             showmeans=True,
                             meanprops={"marker": "o",
                                        "markerfacecolor": "white",
                                        "markeredgecolor": "black",
                                        "markersize": "8"},
                             linewidth=1,
                             showfliers=False)  # ,
            # palette="tab10")

            ax.set_ylabel("Duration (s)", fontsize=24, labelpad=15)
            ax.set_xlabel("Task name", fontsize=24, labelpad=15)
            plt.xticks(fontsize=20, rotation=20)
            plt.yticks(fontsize=21, rotation=0)

            ax.set_title("Task Durations", fontsize=20)
            legend = plt.legend(title='Agent Name', loc='upper left', fontsize=20)
            legend.get_title().set_fontsize('22')

            ax.figure.set_size_inches(17, 8)

            # plt.savefig(self.results_folder_path + "task_durations.png", bbox_inches='tight')

            plt.figure()
            plt.show()

        job_for_another_core = multiprocessing.Process(target=plot_a_graph, args=())
        job_for_another_core.start()

        return SetBoolResponse(True, "ok")

    def checkPickPlaceWithAgent(self, task_name, agent):
        """Method for replacing task name for task performable by both agents
        Args:
            agent (str): task name in db
            agent (str): agent name in db

        Returns:
            str: task name with agent
        """
        if task_name == "pick_blue_box":
            return task_name + "_" + agent
        else:
            return task_name

    def getPaperTaskName(self, task):
        """Method for replacing task names (for charts inherent in the paper)

        Args:
            agent (str): task name in db

        Returns:
            str: Paper task name
        """
        KNOWN_TASK_NAME = {"pick_blue_box_human_right_arm": "Pick Blue Box (H)",
                           "place_blue_box_human_right_arm": "Place Blue Box (H)",
                           "pick_blue_box_ur5_on_guide": "Pick Blue Box (R)",
                           "place_blue_box_ur5_on_guide": "Place Blue Box (R)",
                           "place_blue_box_ur5_on_guide": "Place Blue Box",
                           "pick_white_box": "Pick White Box",
                           "place_white_box": "Place White Box",
                           "pick_orange_box": "Pick Orange Box",
                           "place_orange_box": "Place Orange Box",
                           "probe": "Probe_circuit"
                           }
        task_name = KNOWN_TASK_NAME.get(task, task)
        task_name = task_name.split('_')
        task_name = ' '.join(singol_word.capitalize() for singol_word in task_name)
        print(task_name)
        return task_name
        # if task == "pick_blue_box_human_right_arm":
        #     task_name = "Pick Blue Box (H)"
        # elif task == "place_blue_box_human_right_arm":
        #     task_name = "Place Blue Box (H)"
        # elif task == "pick_blue_box_ur5_on_guide":
        #     task_name = "Pick Blue Box (R)"
        # elif task == "place_blue_box_ur5_on_guide":
        #     task_name = "Place Blue Box (R)"
        # elif task == "place_blue_box_ur5_on_guide":
        #     task_name = "Place Blue Box"
        # elif task == "pick_white_box":
        #     task_name = "Pick White Box"
        # elif task == "place_white_box":
        #     task_name = "Place White Box"
        # elif task == "pick_orange_box":
        #     task_name = "Pick Orange Box"
        # elif task == "place_orange_box":
        #     task_name = "Place Orange Box"
        # return task_name

    def getPaperAgentName(self, agent):
        """Method for replacing agent names (for charts inherent in the paper)

        Args:
            agent (str): agent name in db

        Returns:
            str: Paper agent name
        """
        if agent == "ur5_on_guide":
            agent_label = "Robot"
        elif agent == "human_right_arm":
            agent_label = "Human"
        return agent_label

    def model(self, task_data, main_task_duration, concurrent_task_name):
        """Bayesian Model function

        Args:
            task_data (DataFrame): task_data is a pandas dataframe containing delta informations
            main_task_duration (DataFrame): task duration (observations)
            concurrent_task_name (str): Name of the concurrent task
        """
        # concurrent_task_name = task_data.columns.get_values()[:-1]
        concurrent_synergy = dict()
        mean = 0.
        for id, concurrent_task in enumerate(concurrent_task_name):
            # concurrent_synergy[concurrent_task] = pyro.sample(concurrent_task, dist.Gamma(50.0,50.0))
            concurrent_synergy[concurrent_task] = pyro.sample(concurrent_task, dist.LogNormal(0, 0.5))

            mean += concurrent_synergy[concurrent_task] * task_data[:, id]
        sigma = pyro.sample("sigma", dist.Uniform(0., 2.))

        with pyro.plate("data", len(main_task_duration)):
            pyro.sample("obs", dist.Normal(mean, sigma), obs=main_task_duration)

    def guide(self, main_task, task_data, main_task_duration):
        pass

    def computeBayesianDynamicRisk(self, request):
        """Method for creating a collection of results with added the concurrent tasks

        Args:
            request (SetBoolRequest): _description_

        Returns:
            SetBoolResponse: _description_
        """
        t_start = rospy.Time.now()

        # Delete older dynamic risk collection
        try:
            self.coll_interaction.delete_many({})
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        available_stats = {"average_task_duration": "expected_duration", "min_task_duration": "min",
                           "median_task_duration": "median"}

        used_stat_index = available_stats["average_task_duration"]
        if used_stat_index == "median":
            try:
                self.addMedianToTaskStats()
            except pymongo.errors.AutoReconnect:
                rospy.logerr(CONNECTION_LOST)
                return SetBoolResponse(False, NOT_SUCCESSFUL)

        # Create collection with results task + task mean information
        if not self.createUtilsResults():
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        # Pipeline for: retriving task name, unwinded for agents, each task only 1 agents
        pipeline_agents_task_name = [
            {
                '$unwind': {
                    'path': '$agent',
                    'preserveNullAndEmptyArrays': False
                }
            }, {
                '$project': {
                    '_id': 0,
                    'name': 1,
                    'agent': 1
                }
            }
        ]

        try:
            cursor_task_properties = self.coll_skills.aggregate(pipeline_agents_task_name)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        task_index = dict()  # It will be as: {"agent_name":[list with all tasks that it can perform],..}
        for single_task in cursor_task_properties:
            if "name" in single_task.keys() and "agent" in single_task.keys():  # Ensure it has nedded attributes
                # if not single_task["name"] == "end" or not single_task["name"] == "go_home":  # Excule task end (not interesting)
                if single_task["name"] != "end" and single_task["name"] != "go_home":
                    if single_task["agent"] not in task_index:  # If not exist already that agents
                        task_index[single_task["agent"]] = set()  # A SET for each agent to ensure unique task
                    task_index[single_task["agent"]].add(single_task["name"])

        task_index = {key: list(values) for key, values in
                      task_index.items()}  # Convert agents tasks from set to list (for have index)

        rospy.loginfo("Dizionario: agente->task list: ")
        rospy.loginfo(task_index)

        # input("wait...")

        agents = list(task_index.keys())

        if len(agents) > 2:
            rospy.loginfo(RED + "There are more than 2 agents in task properties" + END)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        # Retrive results with their concurrent tasks information
        pipeline = Pipeline.dynamicRiskPipeline(self.utils_results_name)

        parallelism_type = [PARTIAL_INITIAL, PARTIAL_FINAL, INNER, OUTER]

        try:
            t_start = rospy.Time.now()
            results = self.coll_utils_results.aggregate(pipeline)
            t_end = (rospy.Time.now() - t_start).to_sec()
            rospy.loginfo("Computational time for dynamic risk" + str(t_end))
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False, NOT_SUCCESSFUL)

        rospy.loginfo(GREEN + "Agents: {}".format(agents) + END)
        sns.set_theme()
        for task_group in results:  # A task_group contains vector of all (task_j, agent_i) same task computed by same agent
            # input("New task group...")    #usefull for debug

            # Number of rows of regression matrix
            n_rows = len(task_group['grouped_task_agent'])

            # print(agents)

            # Compute other agents different from current task agent
            agent = task_group["_id"][1]  # First element of _id is name
            concurrent_agent = set(agents).difference(
                set([agent])).pop()  # If only one agent ok, otherwise there is also others agents in the set.

            rospy.loginfo("Principal agent: {}".format(agent))
            rospy.loginfo("Concurrent agent: {}".format(concurrent_agent))

            # for concurrent_agent in other_agents:                   #If more than one agents

            # Number of column of regression matrix = concurrent_agent task
            n_col = len(task_index[concurrent_agent])

            # Initialize regression matrix

            regression_mat = np.empty((0, n_col))
            known_vect = np.empty((0, 1))

            rospy.loginfo(GREEN + "Fondamental task: {}".format(task_group["_id"][0]) + END)

            self.resetConcurrentTaskCounters()

            for row, single_task in enumerate(task_group['grouped_task_agent']):
                add_task_to_regmat = False
                row_vect = np.zeros((1, n_col))

                p_initial_task = single_task['partial_task_initial']
                p_final_task = single_task['partial_task_final']
                # inner_tasks = single_task['inner_task']
                global_outer_task = single_task['outer_task']

                overlapping_tot = 0.0
                for parallelism_task_type in parallelism_type:
                    task = single_task[parallelism_task_type]
                    if parallelism_task_type != INNER and len(task) > 1:
                        rospy.loginfo(RED + "More than one {} task".format(parallelism_task_type) + END)
                        raise Exception("More than one {} task".format(parallelism_task_type))
                    if task:
                        for parallel_task in task:  # per i task inner possono essere più di uno
                            if parallel_task["outcome"] == 1:
                                overlapping_time = self.computeOverlappingRatio(parallelism_task_type,
                                                                                single_task["t_start"],
                                                                                single_task["t_end"],
                                                                                parallel_task["t_start"],
                                                                                parallel_task["t_end"])
                                delta = overlapping_time / parallel_task["delta_time"]
                                parallel_task_stat_cost = parallel_task["task_mean_informations"][0][used_stat_index]

                                col_index_reg_mat = task_index[concurrent_agent].index(parallel_task["name"])
                                row_vect[0, col_index_reg_mat] += delta * parallel_task_stat_cost
                                overlapping_tot += overlapping_time

                                add_task_to_regmat = True
                            self.updateConcurrentTaskCounters(parallel_task["name"], parallel_task["outcome"])

                if add_task_to_regmat:
                    # Nota TODO solo se almeno uno di quelli sopra
                    regression_mat = np.append(regression_mat, row_vect, axis=0)

                    known_vect = np.append(known_vect, overlapping_tot)

            print(regression_mat)
            print(known_vect)
            rospy.loginfo(YELLOW + "****************" + END)
            # if agent == "human_right_arm":
            # clf = IsolationForest(n_estimators=20, warm_start=True)
            # estimator=clf.fit(known_vect.reshape(-1,1))  # fit 10 trees
            # check = estimator.decision_function(known_vect.reshape(-1,1))
            # outliers = check > 0
            #
            # # outliers=np.abs(known_vect-np.mean(known_vect))<np.std(known_vect)*1.8
            # # outliers = known_vect<15
            # print(outliers)
            # regression_mat=regression_mat[outliers,:]
            # known_vect = known_vect[outliers]

            # print(outliers)
            rospy.loginfo(YELLOW + "****************" + END)

            # print(regression_mat.shape)
            # print(known_vect.shape)

            # t_start=rospy.Time.now()
            # try:
            #     lstsq_results=np.linalg.lstsq(regression_mat, known_vect, rcond=None)
            #     print(lstsq_results)
            #     dynamic_risk = lstsq_results[0]
            # except np.linalg.LinAlgError:
            #     rospy.loginfo(RED + "Least square does not converge" + END)
            #     return SetBoolResponse(False,NOT_SUCCESSFUL)        

            print("**************")
            (row, col) = regression_mat.shape
            dizionario = dict()
            formula_string = ""
            for index in range(0, col):
                dizionario[task_index[concurrent_agent][index]] = regression_mat[:, index]
                formula_string += task_index[concurrent_agent][index]
                if index < col - 1:
                    formula_string += " + "
            rospy.loginfo(formula_string)
            # input("Look at formula")  #usefull for debug   
            dizionario["duration"] = known_vect
            print(dizionario)

            dataF = pd.DataFrame(dizionario)

            # print(dataF.columns.values[:-1])
            concurrent_task_name = dataF.columns.values[:-1]
            print("--- Concurrent task name ---")
            print(concurrent_task_name)
            train = torch.tensor(dataF.values, dtype=torch.float)

            task_data, main_task_duration = train[:, 0:-1], train[:, -1]
            nuts_kernel = NUTS(self.model)
            mcmc = MCMC(nuts_kernel, num_samples=1000, warmup_steps=200)
            mcmc.run(task_data, main_task_duration, concurrent_task_name)
            hmc_samples = {k: v.detach().cpu().numpy() for k, v in mcmc.get_samples().items()}

            print("task_group")
            print(task_group)
            fig, axs = plt.subplots(nrows=2, ncols=2, figsize=(12, 10))
            TODO: Bug fix in this section
            # chart_title = TASK_NAME_FOR_CHART[task_group["_id"][0]]
            #
            # chart_agent_name = AGENT_NAME_FOR_CHART[agent]
            # fig.suptitle(
            #     "Marginal Posterior density - Regression Coefficients. \n Main agent: {}, Main Task: {}".format(
            #         chart_agent_name,
            #         chart_title),
            #     fontsize=16)
            #
            # with open(self.results_folder_path + f"ISO5_synergy_estimation_{chart_agent_name}.yaml", 'w') as outfile:
            #     yaml.dump([hmc_samples, concurrent_task_name], outfile, default_flow_style=False)
            # with open(self.results_folder_path + f"ISO5_synergy_estimation_{chart_agent_name}.pkl", 'wb') as fp:
            #     pickle.dump([hmc_samples, concurrent_task_name], fp)
            #
            # # np.save(self.results_folder_path + f"ISO5_synergy_estimation_{chart_agent_name}.npy", hmc_samples)
            # sns.set_theme()
            # print(concurrent_task_name)
            # for i, ax in enumerate(axs.reshape(-1)):
            #     site = concurrent_task_name[i]
            #
            #     # sns.distplot(svi_samples[site], ax=ax, label="SVI (DiagNormal)")
            #     sns.set_theme()
            #     sns.distplot(hmc_samples[site], ax=ax, label="HMC")
            #     ax_title = TASK_NAME_FOR_CHART[site]
            #     ax.set_title(ax_title)
            #     ax.set_xlabel("Synergy value")
            # handles, labels = ax.get_legend_handles_labels()
            # fig.legend(handles, labels, loc='upper right');
            # fig.tight_layout()
            # # plt.show()
            # sns.set_theme()
            # plt.savefig(self.results_folder_path + "ISO5_Bayesian_Gamma_dynamic_risk_agent_" + concurrent_agent + "_" +
            #             task_group["_id"][0] + ".png", bbox_inches='tight')
            # plt.savefig(self.results_folder_path + "ISO5_Bayesian_Gamma_dynamic_risk_agent_" + concurrent_agent + "_" +
            #             task_group["_id"][0] + ".pdf", bbox_inches='tight')
            #
            # reg_model = smf.ols(formula="duration ~ " + formula_string + " -1 ", data=dataF)
            # reg_result = reg_model.fit()
            # print(reg_result.summary())

            print("QUIIIII")
            print(task_index[concurrent_agent])

            print("QUaaaaaa")
            print(self.concurrent_task_counters)

            # #Here agent and concurrent_agent change role for dynamic risk
            for index, task in enumerate(task_index[concurrent_agent]):
                counter, success_rate = self.getConcurrentTaskStatistics(task)
                print(task)
                # TODO: Has to work
                # print(reg_result.bse[task])
                print(hmc_samples[task])
                dynamic_risk = pd.DataFrame(hmc_samples[task])[0].describe()
                try:
                    # results = self.coll_utils_results.aggregate(pipeline)
                    self.coll_interaction.insert_one({"agent": concurrent_agent,
                                                      "concurrent_agent": agent,
                                                      "agent_skill": task,
                                                      "concurrent_skill": task_group["_id"][0],
                                                      "success_rate": success_rate,
                                                      "dynamic_risk": dynamic_risk["mean"],
                                                      "std_err": dynamic_risk["std"],
                                                      "counter": counter})
                except pymongo.errors.AutoReconnect:
                    rospy.logerr(CONNECTION_LOST)
                    return SetBoolResponse(False, NOT_SUCCESSFUL)

            rospy.loginfo(GREEN + "Fondamental task:" + END)
            print(single_task["name"])
            print(task_index[concurrent_agent])
            # print(dynamic_risk)
            # input("Look at results")

            print((rospy.Time.now() - t_start).to_sec())

        return SetBoolResponse(True, SUCCESSFUL)

    def getTaskSynergy(self, request):
        rospy.loginfo(SERVICE_CALLBACK.format("get_task_synergies"))

        synergies_response = TaskSynergiesResponse()
        synergies_response.synergies = []
        try:
            query_result = self.coll_interaction.find({"agent": request.agent,
                                                       "agent_skill": request.task_name},
                                                      {"concurrent_agent": 1,
                                                       "concurrent_skill": 1,
                                                       "dynamic_risk": 1,
                                                       "std_err": 1,
                                                       "success_rate": 1,
                                                       "_id": 0})
            for single_task_synergy in query_result:
                if not all(synergy_info in single_task_synergy for synergy_info in ["concurrent_skill",
                                                                                    "concurrent_agent",
                                                                                    "dynamic_risk",
                                                                                    "std_err",
                                                                                    "success_rate"]):
                    rospy.logerr(RED + "Statistics not present in db" + END)
                    raise Exception
                task_synergy = TaskSynergy()
                task_synergy.task_name = single_task_synergy["concurrent_skill"]
                task_synergy.agent = single_task_synergy["concurrent_agent"]
                task_synergy.synergy = single_task_synergy["dynamic_risk"]
                task_synergy.std_err = single_task_synergy["std_err"]
                task_synergy.success_rate = single_task_synergy["success_rate"]
                # TODO: Before append check that is unique
                synergies_response.synergies.append(task_synergy)
        except pymongo.errors.AutoReconnect:  # Db connection failed
            rospy.logerr(CONNECTION_LOST)
        return synergies_response


def main():
    rospy.init_node("mongo_statistics")

    try:
        db_name = rospy.get_param("~mongo_database")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("mongo_database") + END)
        return 0
    try:
        coll_properties_name = rospy.get_param("~mongo_collection_tasks")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("mongo_collection_tasks") + END)
    try:
        coll_results_name = rospy.get_param("~mongo_collection_results")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("mongo_collection_results") + END)
        return 0
    try:
        coll_duration_name = rospy.get_param("~coll_duration_name")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("coll_duration_name") + END)
        return 0
    try:
        coll_risk_name = rospy.get_param("~coll_risk_name")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("coll_risk_name") + END)
        return 0
    try:
        fig_folder = rospy.get_param("~fig_folder_path")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("fig_folder_path") + END)
        fig_folder = "file"
        rospy.logerr("fig_folder set to: " + fig_folder)

    # db_name = "agents_synergy"
    # coll_properties_name = "tasks_properties_test"
    # coll_results_name = "sinergy_results_test_long"  # "results_test"
    #
    # coll_duration_name = "durate"
    #
    # # coll_risk_name = "dynamics_long_test_outliers_3"
    # coll_risk_name = "sinergie"
    # fig_folder = "/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_statistics/file/"

    try:
        mongo_statistics = MongoStatistics(db_name, coll_properties_name, coll_results_name, coll_duration_name,
                                           coll_risk_name, fig_folder)
    except:
        return 0  # Connection to db failed

    # Rosservice

    rospy.Service(COMPUTE_DURATION_SERVICE, SetBool, mongo_statistics.computeDurations)
    rospy.Service(COMPUTE_DYNAMIC_RISK_SERVICE, SetBool, mongo_statistics.computeDynamicRisk)
    # COMPUTE_DYNAMIC_RISK_SERVICE_MIN = "mongo_statistics/compute_dynamic_risk_min"
    # rospy.Service(COMPUTE_DYNAMIC_RISK_SERVICE_MIN,SetBool,mongo_statistics.computeDynamicRiskMin)

    rospy.Service(COMPUTE_DYNAMIC_RISK_WITH_UNC_SERVICE, SetBool, mongo_statistics.computeDynamicRiskWithUncertainty)
    rospy.Service(COMPUTE_BAYESIAN_DYNAMIC_RISK_SERVICE, SetBool, mongo_statistics.computeBayesianDynamicRisk)

    # rospy.Service("prova_regressione_median",SetBool,mongo_statistics.computeDynamicRiskWithUncertaintyMedian) 

    rospy.Service(TASK_DURATION_CHART_SERVICE, SetBool, mongo_statistics.taskDurationChart)
    rospy.Service(TASK_DURATION_BY_GROUPING_CHART_SERVICE, SetBool, mongo_statistics.groupedDurationChart)
    rospy.Service(SYNERGY_MATRIX_CHART_SERVICE, SetBool, mongo_statistics.dynamicRiskChart)
    rospy.Service(SYNERGY_UNCERTAINTY_CHART_SERVICE, SetBool, mongo_statistics.singleDRCoefficientChart)
    rospy.Service(TIMELINE_CHART_SERVICE, SetBool, mongo_statistics.drawTimeline)

    rospy.Service("mongo_statistics/get_task_synergies", TaskSynergies, mongo_statistics.getTaskSynergy)

    rospy.loginfo(READY)
    rospy.spin()


if __name__ == "__main__":
    main()
