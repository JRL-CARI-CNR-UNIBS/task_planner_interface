#! /usr/bin/env python3

import rospy
from pymongo import MongoClient
import pymongo.errors
import seaborn as sns
import matplotlib.pyplot as plt




PARAM_NOT_DEFINED_ERROR = "Parameter: {} not defined"

CONNECTION_FAILED = "Connection to db failed: Request Timeout"
CONNECTION_OK = "Connection to db executed"
AGGREGATION_OK = "Aggregation computed correctly"

def main():
    # rospy.init_node("mongo_statistics")
    
    # try:
    #     db_name=rospy.get_param("~mongo_database")
    # except KeyError:
    #     rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("mongo_database") + END)
    #     return 0
    # try:
    #     coll_properties_name=rospy.get_param("~mongo_collection_tasks")   
    # except KeyError:   
    #     rospy.logerr(RED + PARAM_NOT_DEFINEtask
    # try:
    #     coll_duration_name=rospy.get_param("~coll_duration_name")   
    # except KeyError:
    #     rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("coll_duration_name") + END)
    #     return 0
    # try:
    #     coll_risk_name=rospy.get_param("~coll_risk_name")   
    # except KeyError:        
    #     rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("coll_risk_name") + END)
    #     return 0    
    # try:
    #     fig_folder=rospy.get_param("~fig_folder_path")
    # except KeyError:
    #     rospy.logerr(PARAM_NOT_DEFINED_ERROR.format("fig_folder_path"))
    #     fig_folder = "file"
    #     rospy.logerr("fig_folder set to: " + fig_folder)
    #

    db_name = "agents_synergy"
    coll_results_name = "utils_results"

    # fig_folder="/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_statistics/file/"

    # try:
    #     mongo_statistics = MongoStatistics(db_name,coll_properties_name,coll_results_name,coll_duration_name,coll_risk_name,fig_folder)
    # except:
    #     return 0     #Connection to db failed

    client = MongoClient(serverSelectionTimeoutMS=5000)  # 5 seconds of maximum connection wait
    try:
        client.server_info()
        rospy.loginfo(CONNECTION_OK)
    except pymongo.errors.ServerSelectionTimeoutError:
        rospy.logerr(CONNECTION_FAILED)
        return 0

    db = client[db_name]
    coll_results = db[coll_results_name]


    pipeline = [
        {
            '$group': {
                '_id': '$recipe',
                't_start_min': {
                    '$min': '$t_start'
                },
                't_end_max': {
                    '$max': '$t_end'
                }
            }
        }, {
            '$addFields': {
                'recipe_duration': {
                    '$subtract': [
                        '$t_end_max', '$t_start_min'
                    ]
                }
            }
        }
    ]
    try:
        results = coll_results.aggregate(pipeline)
        rospy.loginfo(AGGREGATION_OK)
    except pymongo.errors.AutoReconnect:
        return 0
    recipe_durations = []
    recipe_info = {}
    for recipe in results:
        recipe_info[recipe['_id']] = recipe['recipe_duration']
        recipe_durations.append(recipe["recipe_duration"])

    sns.set_style("darkgrid")
    sns.lineplot(data = recipe_durations)
    plt.xlabel("Recipe Number")
    plt.ylabel("Recipe Duration")
    plt.title("Recipe Durations")
    plt.figure()
    ax = sns.histplot(data = recipe_durations)
    ax.bar_label(ax.containers[0])
    plt.xlabel("Recipe Number")
    plt.ylabel("Recipe Duration")
    plt.title("Recipe Durations")

    #### Get Synergy #############
    results = db["dynamics_long_test_outliers_2"].find({"agent": "ur5_on_guide"},
                                                       {"agent_skill", "concurrent_skill", "dynamic_risk"})
    synergy = {}
    for synergy_term in results:
        synergy[(synergy_term["agent_skill"],synergy_term["concurrent_skill"])] = synergy_term["dynamic_risk"]

    #### Compute recipe index ####
    plt.show()
    pipeline = [
        {
            '$group': {
                '_id': {
                    'recipe_name': '$recipe',
                    'agent': '$agent'
                },
                'agent_task': {
                    '$push': '$$ROOT'
                }
            }
        }, {
        '$group': {
            '_id': '$_id.recipe_name',
            'tasks': {
                '$push': '$$ROOT'
            }
        }
    }
    ]
    try:
        results = coll_results.aggregate(pipeline)
        rospy.loginfo(AGGREGATION_OK)
    except pymongo.errors.AutoReconnect:
        return 0

    recipe_durations = []
    recipe_costs = {}
    for recipe in results:

        agent_tasks = {}
        recipe_name = recipe["_id"]
        recipe_costs[recipe_name] = {"cost": 0, "duration": recipe_info[recipe_name]}

        for tasks_grouped in recipe["tasks"]:
            agent = tasks_grouped["_id"]["agent"]
            if not agent in agent_tasks:
                agent_tasks[agent] = []
            for task in tasks_grouped["agent_task"]:
                assert task["recipe"] == recipe_name
                assert task["agent"] == agent

                agent_tasks[agent].append(
                    {"task_name": task["name"], "t_start": task["t_start"], "t_end": task["t_end"]})
        # print(agent_tasks.keys())
        if not len(agent_tasks.keys()) == 2:
            continue
        other_agent = "human_right_arm"
        # print(agent_tasks['ur5_on_guide'])
        for task in agent_tasks["ur5_on_guide"]:
            task_name = task["task_name"]
            for other_agent_task in agent_tasks[other_agent]:
                # print(other_agent_task)
                other_agent_task_name = other_agent_task["task_name"]


                delta = min(other_agent_task["t_end"],task["t_end"]) - max(other_agent_task["t_start"],task["t_start"])
                other_agent_task_duration = other_agent_task["t_end"] - other_agent_task["t_start"]
                assert other_agent_task_duration > 0
                if delta > 0:
                    # print(delta/other_agent_task_duration)
                    # if delta/other_agent_task_duration == 1:
                        # print(other_agent_task)
                    # print(synergy[(task_name,other_agent_task_name)])
                    recipe_costs[recipe_name]["cost"] += delta/other_agent_task_duration * synergy[(task_name,other_agent_task_name)]
    # print(recipe_costs)
    sort_recipe_costs = {k: v for k, v in sorted(recipe_costs.items(), key=lambda item: item[1]["cost"])}
    print(sort_recipe_costs)

if __name__ == "__main__":
    main()