#! /usr/bin/env python3

import rospy

from std_srvs.srv import SetBool,SetBoolResponse

from pymongo import MongoClient
import pymongo.errors

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

import numpy as np
import pprint
import os
import copy

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
COMPUTE_DYNAMIC_RISK_SERVICE_V2 = "mongo_statistics/compute_dynamic_risk_v2"
MAKE_CHART_SERVICE = "/mongo_statistics/make_timeline_chart"
MAKE_DR_CHART_SERVICE = "/mongo_statistics/make_dr_chart"

class MongoStatistics:
    
    def __init__(self,db_name,coll_properties_name,coll_results_name,coll_duration_name,coll_risk_name,results_folder_path):
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
        
        client = MongoClient(serverSelectionTimeoutMS=5000)     # 5 seconds of maximum connection wait
        try:
            client.server_info()
            rospy.loginfo(GREEN + CONNECTION_OK + END)
        except pymongo.errors.ServerSelectionTimeoutError:
            rospy.logerr(RED + CONNECTION_FAILED + END)
            raise
         
        self.db = client[db_name]
        self.coll_skills = self.db[coll_properties_name]
        self.coll_results = self.db[coll_results_name]
        self.coll_interaction = self.db[coll_risk_name]
        self.coll_durations = self.db[coll_duration_name]       

        self.coll_durations_name = coll_duration_name 
        self.coll_results_name =  coll_results_name

        self.utils_results_name = "utils_results"

    
        self.coll_utils_results = self.db[self.utils_results_name]       
        
        self.results_folder_path = results_folder_path
        

    def computeDurations(self,request):
        """_summary_

        Args:
            request (_type_): _description_

        Returns:
            _type_: _description_
        """
        
        rospy.loginfo(SERVICE_CALLBACK.format(COMPUTE_DURATION_SERVICE))
        
        
        pipeline = [
        {
            '$project': {
                'duration': {
                    '$cond': {
                        'if': {
                            '$eq': [
                                '$outcome', 0
                            ]
                        }, 
                        'then': None, 
                        'else': '$duration_real'
                    }
                }, 
                'name': 1, 
                'type': 1, 
                'agent': 1, 
                'outcome': 1
            }
        }, {
            '$group': {
                '_id': {
                    'agent': '$agent', 
                    'name': '$name'
                }, 
                'expected_duration': {
                    '$avg': '$duration'
                }, 
                'duration_stddev': {
                    '$stdDevSamp': '$duration'
                }, 
                'counter_success': {
                    '$sum': '$outcome'
                }, 
                'counter': {
                    '$sum': 1
                }, 
                'success_rate': {
                    '$avg': '$outcome'
                }
            }
        }, {
            '$project': {
                '_id': 0, 
                'name': '$_id.name', 
                'agent': '$_id.agent', 
                'expected_duration': {
                    '$cond': {
                        'if': {
                            '$eq': [
                                '$expected_duration', None
                            ]
                        }, 
                        'then': 0, 
                        'else': '$expected_duration'
                    }
                }, 
                'duration_stddev': {
                    '$cond': {
                        'if': {
                            '$eq': [
                                '$duration_stddev', None
                            ]
                        }, 
                        'then': 0, 
                        'else': '$duration_stddev'
                    }
                }, 
                'success_rate': 1, 
                'counter': 1
            }
        }, {
            '$sort': {
                'name': 1
            }
        },  {
        '$out': self.coll_durations_name
        }
        ]
        
        pipeline=[
        {
            '$project': {
                'duration': {
                    '$cond': {
                        'if': {
                            '$eq': [
                                '$outcome', 0
                            ]
                        }, 
                        'then': None, 
                        'else': {
                            '$subtract': [
                                '$t_end', '$t_start'
                            ]
                        }
                    }
                }, 
                'name': 1, 
                'type': 1, 
                'agent': 1, 
                'outcome': 1
            }
        }, {
            '$group': {
                '_id': {
                    'agent': '$agent', 
                    'name': '$name'
                }, 
                'expected_duration': {
                    '$avg': '$duration'
                }, 
                'duration_stddev': {
                    '$stdDevSamp': '$duration'
                }, 
                'counter_success': {
                    '$sum': '$outcome'
                }, 
                'counter': {
                    '$sum': 1
                }, 
                'success_rate': {
                    '$avg': '$outcome'
                }, 
                'min': {
                    '$min': '$duration'
                }, 
                'max': {
                    '$max': '$duration'
                }
            }
        }, {
            '$project': {
                'name': '$_id.name', 
                'agent': '$_id.agent', 
                'expected_duration': {
                    '$cond': {
                        'if': {
                            '$eq': [
                                '$expected_duration', None
                            ]
                        }, 
                        'then': 0, 
                        'else': '$expected_duration'
                    }
                }, 
                'duration_stddev': {
                    '$cond': {
                        'if': {
                            '$eq': [
                                '$duration_stddev', None
                            ]
                        }, 
                        'then': 0, 
                        'else': '$duration_stddev'
                    }
                }, 

                'counter': 1, 
                'success_rate': 1, 
                'min': 1, 
                'max': 1
            }
        }, {
            '$sort': {
                'name': 1
            }
        },{
        '$out': self.coll_durations_name
        }
        ]
        try:
            self.coll_durations.delete_many({})
            results = self.coll_results.aggregate(pipeline)
            rospy.loginfo(DURATION_OK)
            return SetBoolResponse(True,SUCCESSFUL)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        


    def createUtilsResults(self):  
        """Method for creating a collection of results with added task_mean_information and delta_time based on t_end-t_start

        Returns:
            bool: True if no exception occure, False viceversa
        """
        rospy.loginfo(RED + "CHIAMATO UTILS RESULTS" + END)
        # Delete existing collection if it exists
        self.coll_utils_results.delete_many({})
        
        # TODO FIRST CHECK THAT IN DURATION COLLECTION THERE IS SOMETHINF
        #self.coll_durations.count_documents({})
        # self.coll_interaction.delete_many({})
        t_start = rospy.Time.now()
        
        # Pipeline = add Field delta_time and search task mean information from duration document
        pipeline=[                          
        {
            '$addFields': {
                'delta_time': {
                    '$subtract': [
                        '$t_end', '$t_start'
                    ]
                }
            }
        }, {
            '$lookup': {
                'from': self.coll_durations_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$eq': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$name', '$$local_name'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'task_mean_informations'
            }
        }, {
            '$out': self.utils_results_name
        }
        ]
        
        try:
            results = self.coll_results.aggregate(pipeline)
            t_end = rospy.Time.now()
                        
            rospy.loginfo(RED + "Tempo impiegato: " + str((t_end-t_start).to_sec())+ END)

            rospy.loginfo("Concurrent Results ok")
            return True
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return False 
    
    def getConcurrentTaskStatistics(self, task_name):

        return self.concurrent_task_counters[task_name]["counter"] , self.concurrent_task_counters[task_name]["success_counter"]/self.concurrent_task_counters[task_name]["counter"]

    def getCounter(self, task_name):
        pass
    def resetConcurrentTaskCounters(self):
        self.concurrent_task_counters = dict()
        
    def updateConcurrentTaskCounters(self, task_name, task_outcome):
        if task_name not in self.concurrent_task_counters:
            self.concurrent_task_counters[task_name] = {"task_name": task_name, "counter": 0, "success_counter": 0}
        
        self.concurrent_task_counters[task_name]["counter"] += 1
        self.concurrent_task_counters[task_name]["success_counter"] += task_outcome
        

    def computeDynamicRisk(self,request):   #Compute dynamic risk with concurrent and mean information
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
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        # Create collection with results task + task mean information
        if not self.createUtilsResults():
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        # Pipeline for: retriving task name, unwinded for agents, each task only 1 agents
        pipeline_agents_task_name =[
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
            return SetBoolResponse(False,NOT_SUCCESSFUL)
                
        task_index = dict()                                                         # It will be as: {"agent_name":[list with all tasks that it can perform],..}
        for single_task in cursor_task_properties:
            if "name" in single_task.keys() and "agent" in single_task.keys() :     # Ensure it has nedded attributes
                if not single_task["name"] == "end":                                # Excule task end (not interesting)
                    if single_task["agent"] not in task_index:                      # If not exist already that agents
                        task_index[single_task["agent"]] = set()                    # A SET for each agent to ensure unique task
                    task_index[single_task["agent"]].add(single_task["name"])
                                    
        task_index = {key: list(values) for key, values in task_index.items()}       # Convert agents tasks from set to list (for have index)
        
        rospy.loginfo("Dizionario: agente->task list: ")
        rospy.loginfo(task_index)
        rospy.loginfo(RED + "OK FINO TASK RETRIEVE INFO" + END)
        # input("wait...")
        
        agents = list(task_index.keys())
        
        if len(agents)>2:
            rospy.loginfo(RED + "There are more than 2 agents in task properties" + END)
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        ############
        #Retrive results with their concurrent tasks information
        pipeline = [
        {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lte': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }, {
                                        '$gte': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'inner_task'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_start', '$t_end'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$t_end', '$$local_t_end'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'partial_task_initial'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_end', '$t_start'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'partial_task_final'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'outer_task'
            }
        }, {
            '$group': {                     # Il group molto lento 0.1 -> 0.7
                '_id': [
                    '$name', '$agent'
                ], 
                'grouped_task_agent': {
                    '$push': '$$ROOT'
                }
            }
        }
        ]

        try:
            t_start = rospy.Time.now()
            results = self.coll_utils_results.aggregate(pipeline)
            t_end = (rospy.Time.now() - t_start).to_sec()
            rospy.loginfo("Computational time for dynamic risk" + str(t_end))
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        rospy.loginfo(YELLOW + "Iterating results" + END)
        

        rospy.loginfo(GREEN + "Agents: {}".format(agents) + END)
        for task_group in results:          #A task_group contains vector of all (task_j, agent_i) same task computed by same agent
            input("New task group...")
            
            #Number of rows of regression matrix
            n_rows = len(task_group['grouped_task_agent'])
            
            # print(agents)
            
            #Compute other agents different from current task agent
            agent = task_group["_id"][1]                                    #First element of _id is name
            concurrent_agent = list(set(agents).difference(set([agent])))[0]         # If only one agent ok, otherwise there is also others agents in the set.
            
            rospy.loginfo("Principal agent: {}".format(agent))
            rospy.loginfo("Concurrent agent: {}".format(concurrent_agent))
                        
                       
            #for concurrent_agent in other_agents:                   #If more than one agents
            
            #Number of column of regression matrix = concurrent_agent task
            n_col = len(task_index[concurrent_agent])
            
            #Initialize regression matrix
            
            #regression_mat = np.zeros((n_rows,n_col))                       # To change: non è detto che per un task ce ne sia uno parallelo
            #row_vect = np.zeros((1,n_col))
            
            regression_mat = np.empty((0,n_col)) 
            known_vect = np.empty((0,1))
            
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
                row_vect = np.zeros((1,n_col))
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
                #Insert partial initial task
                if p_initial_task:          # Not empty
                    if len(p_initial_task)>1:                                               #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
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
                            t_initial_mean = p_initial_task[0]["task_mean_informations"][0]["expected_duration"] #"task_mean_informations":[{"exp_dur":**}]
                            # print(task_index[concurrent_agent])
                            col_index_reg_mat = task_index[concurrent_agent].index(p_initial_task[0]["name"])
                            
                            #regression_mat[row, col_index_reg_mat] = delta_initial*t_initial_mean
                            
                            row_vect[0,col_index_reg_mat] += delta_initial*t_initial_mean
                            add_task_to_regmat = True
                            
                        self.updateConcurrentTaskCounters(p_initial_task[0]["name"], p_initial_task[0]["outcome"])
                        
                        #print(single_task)
                        #print(p_initial_task)
                        # print(p_initial_task[0]["name"])
                        # print(p_initial_task[0]["agent"])
                        # print(p_initial_task[0]["t_start"])
                        # print(p_initial_task[0]["t_end"])       
                        
                        # print(overlapping_initial_time)
                        # print(delta_initial)
                        # print(t_initial_mean)
                        # print(col_index_reg_mat)
                        
                #Insert partial final task
                if p_final_task:          # Not empty
                    if len(p_final_task)>1:
                        rospy.loginfo(RED + "More than one partial final task" + END)        #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                    else:
                        if p_final_task[0]["outcome"] == 1:          
                            rospy.loginfo(RED + "Final task: {}".format(p_final_task[0]["name"]) + END)

                            overlapping_final_time = single_task["t_end"] - p_final_task[0]["t_start"]
                            delta_final = overlapping_final_time / p_final_task[0]["delta_time"]
                            # print(p_final_task[0]["task_mean_informations"])
                            # print(p_final_task[0]["task_mean_informations"][0])
                            # print(p_final_task[0]["task_mean_informations"][0]["expected_duration"])
                            t_final_mean = p_final_task[0]["task_mean_informations"][0]["expected_duration"] #"task_mean_informations":[{"exp_dur":**}]
                            # print(concurrent_agent)
                            col_index_reg_mat = task_index[concurrent_agent].index(p_final_task[0]["name"])
                            
                            #regression_mat[row, col_index_reg_mat] = delta_final*t_final_mean
                            row_vect[0,col_index_reg_mat] += delta_final*t_final_mean
                            add_task_to_regmat = True

                        self.updateConcurrentTaskCounters(p_final_task[0]["name"], p_final_task[0]["outcome"])
                        #print(single_task)
                        
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
                        if inner_task["outcome"] == 1:               #QUesto [0] non sono sicuro
                            col_index_reg_mat = task_index[concurrent_agent].index(inner_task["name"])
                            # print("another inner tasks")
                            print(inner_task["name"])
                            # print(inner_task["task_mean_informations"])
                            # print(inner_task["task_mean_informations"][0]["expected_duration"])                        
                            # print(inner_task["task_mean_informations"][0])
                            #regression_mat[row, col_index_reg_mat] = inner_task["task_mean_informations"][0]["expected_duration"]
                            
                            row_vect[0,col_index_reg_mat] += inner_task["task_mean_informations"][0]["expected_duration"]
                            
                            overlapping_inner_tasks += inner_task["delta_time"]
                            add_task_to_regmat = True
                        
                        self.updateConcurrentTaskCounters(inner_task["name"], inner_task["outcome"])
                        # input("guarda inner task")
                        # print(inner_task["name"])
                
                if global_outer_task:
                    if len(global_outer_task)>1:
                        rospy.loginfo(RED + "More than one global outside final task" + END)        #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                    else:
                        if global_outer_task[0]["outcome"] == 1:          
                            rospy.loginfo(RED + "Global outside task: {}".format(global_outer_task[0]["name"]) + END)

                            delta_global_outside = single_task["delta_time"] / global_outer_task[0]["delta_time"]
                            
                            t_global_outside_task_mean = global_outer_task[0]["task_mean_informations"][0]["expected_duration"] #"task_mean_informations":[{"exp_dur":**}]
                            
                            col_index_reg_mat = task_index[concurrent_agent].index(global_outer_task[0]["name"])
                            #regression_mat[row, col_index_reg_mat] = delta_global_outside * t_global_outside_task_mean     # delta_time is the real duration of "inner task"

                            row_vect[0,col_index_reg_mat] += delta_global_outside * t_global_outside_task_mean

                            overlapping_global_outer_task = single_task["delta_time"]                                      # All the little task   |---|
                            add_task_to_regmat = True                                                                                                   # |--------|
                        self.updateConcurrentTaskCounters(global_outer_task[0]["name"], global_outer_task[0]["outcome"])
                print(row_vect)
                print(overlapping_initial_time)
                print(overlapping_inner_tasks)
                print(overlapping_final_time)
                print(overlapping_global_outer_task)
                if add_task_to_regmat:
                    #Nota TODO solo se almeno uno di quelli sopra
                    regression_mat = np.append(regression_mat, row_vect,axis=0)
                    # print("------------------")
                    # print(regression_mat)
                    # print("------------------")
                    
                    known_vect = np.append(known_vect, overlapping_initial_time + overlapping_final_time + overlapping_inner_tasks + overlapping_global_outer_task)
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
            
            t_start=rospy.Time.now()
            try:
                lstsq_results=np.linalg.lstsq(regression_mat, known_vect, rcond=None)
                print(lstsq_results)
                dynamic_risk = lstsq_results[0]
            except np.linalg.LinAlgError:
                rospy.loginfo(RED + "Least square does not converge" + END)
                return SetBoolResponse(False,NOT_SUCCESSFUL)        
            
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
                    return SetBoolResponse(False,NOT_SUCCESSFUL)
                
            rospy.loginfo(GREEN + "Fondamental task:" + END)
            print(single_task["name"])
            print(task_index[concurrent_agent])
            print(dynamic_risk)
            input("Leggi i risultati...")
            # for k in coefficient:
            #     input("Prossimo k")
            #     print(k)
                
            print((rospy.Time.now()-t_start).to_sec())
            
        return SetBoolResponse(True,SUCCESSFUL)


    def computeDynamicRiskMin(self,request):   #Compute dynamic risk with concurrent and mean information
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
                return SetBoolResponse(False,NOT_SUCCESSFUL)
            
            # Create collection with results task + task mean information
            if not self.createUtilsResults():
                return SetBoolResponse(False,NOT_SUCCESSFUL)
            
            # Pipeline for: retriving task name, unwinded for agents, each task only 1 agents
            pipeline_agents_task_name =[
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
                return SetBoolResponse(False,NOT_SUCCESSFUL)
                    
            task_index = dict()                                                         # It will be as: {"agent_name":[list with all tasks that it can perform],..}
            for single_task in cursor_task_properties:
                if "name" in single_task.keys() and "agent" in single_task.keys() :     # Ensure it has nedded attributes
                    if not single_task["name"] == "end":                                # Excule task end (not interesting)
                        if single_task["agent"] not in task_index:                      # If not exist already that agents
                            task_index[single_task["agent"]] = set()                    # A SET for each agent to ensure unique task
                        task_index[single_task["agent"]].add(single_task["name"])
                                        
            task_index = {key: list(values) for key, values in task_index.items()}       # Convert agents tasks from set to list (for have index)
            
            rospy.loginfo("Dizionario: agente->task list: ")
            rospy.loginfo(task_index)
            rospy.loginfo(RED + "OK FINO TASK RETRIEVE INFO" + END)
            # input("wait...")
            
            agents = list(task_index.keys())
            
            if len(agents)>2:
                rospy.loginfo(RED + "There are more than 2 agents in task properties" + END)
                return SetBoolResponse(False,NOT_SUCCESSFUL)
            
            ############
            #Retrive results with their concurrent tasks information
            pipeline = [
            {
                '$lookup': {
                    'from': self.utils_results_name, 
                    'let': {
                        'local_name': '$name', 
                        'local_agent': '$agent', 
                        'local_recipe': '$recipe', 
                        'local_t_start': '$t_start', 
                        'local_t_end': '$t_end'
                    }, 
                    'pipeline': [
                        {
                            '$match': {
                                '$expr': {
                                    '$and': [
                                        {
                                            '$ne': [
                                                '$agent', '$$local_agent'
                                            ]
                                        }, {
                                            '$eq': [
                                                '$recipe', '$$local_recipe'
                                            ]
                                        }, {
                                            '$lte': [
                                                '$$local_t_start', '$t_start'
                                            ]
                                        }, {
                                            '$gte': [
                                                '$$local_t_end', '$t_end'
                                            ]
                                        }
                                    ]
                                }
                            }
                        }, {
                            '$project': {
                                'stock_item': 0, 
                                '_id': 0
                            }
                        }
                    ], 
                    'as': 'inner_task'
                }
            }, {
                '$lookup': {
                    'from': self.utils_results_name, 
                    'let': {
                        'local_name': '$name', 
                        'local_agent': '$agent', 
                        'local_recipe': '$recipe', 
                        'local_t_start': '$t_start', 
                        'local_t_end': '$t_end'
                    }, 
                    'pipeline': [
                        {
                            '$match': {
                                '$expr': {
                                    '$and': [
                                        {
                                            '$ne': [
                                                '$agent', '$$local_agent'
                                            ]
                                        }, {
                                            '$eq': [
                                                '$recipe', '$$local_recipe'
                                            ]
                                        }, {
                                            '$lt': [
                                                '$$local_t_start', '$t_end'
                                            ]
                                        }, {
                                            '$gt': [
                                                '$$local_t_start', '$t_start'
                                            ]
                                        }, {
                                            '$lt': [
                                                '$t_end', '$$local_t_end'
                                            ]
                                        }
                                    ]
                                }
                            }
                        }, {
                            '$project': {
                                'stock_item': 0, 
                                '_id': 0
                            }
                        }
                    ], 
                    'as': 'partial_task_initial'
                }
            }, {
                '$lookup': {
                    'from': self.utils_results_name, 
                    'let': {
                        'local_name': '$name', 
                        'local_agent': '$agent', 
                        'local_recipe': '$recipe', 
                        'local_t_start': '$t_start', 
                        'local_t_end': '$t_end'
                    }, 
                    'pipeline': [
                        {
                            '$match': {
                                '$expr': {
                                    '$and': [
                                        {
                                            '$ne': [
                                                '$agent', '$$local_agent'
                                            ]
                                        }, {
                                            '$eq': [
                                                '$recipe', '$$local_recipe'
                                            ]
                                        }, {
                                            '$gt': [
                                                '$$local_t_end', '$t_start'
                                            ]
                                        }, {
                                            '$lt': [
                                                '$$local_t_end', '$t_end'
                                            ]
                                        }, {
                                            '$lt': [
                                                '$$local_t_start', '$t_start'
                                            ]
                                        }
                                    ]
                                }
                            }
                        }, {
                            '$project': {
                                'stock_item': 0, 
                                '_id': 0
                            }
                        }
                    ], 
                    'as': 'partial_task_final'
                }
            }, {
                '$lookup': {
                    'from': self.utils_results_name, 
                    'let': {
                        'local_name': '$name', 
                        'local_agent': '$agent', 
                        'local_recipe': '$recipe', 
                        'local_t_start': '$t_start', 
                        'local_t_end': '$t_end'
                    }, 
                    'pipeline': [
                        {
                            '$match': {
                                '$expr': {
                                    '$and': [
                                        {
                                            '$ne': [
                                                '$agent', '$$local_agent'
                                            ]
                                        }, {
                                            '$eq': [
                                                '$recipe', '$$local_recipe'
                                            ]
                                        }, {
                                            '$lt': [
                                                '$$local_t_end', '$t_end'
                                            ]
                                        }, {
                                            '$gt': [
                                                '$$local_t_start', '$t_start'
                                            ]
                                        }
                                    ]
                                }
                            }
                        }, {
                            '$project': {
                                'stock_item': 0, 
                                '_id': 0
                            }
                        }
                    ], 
                    'as': 'outer_task'
                }
            }, {
                '$group': {                     # Il group molto lento 0.1 -> 0.7
                    '_id': [
                        '$name', '$agent'
                    ], 
                    'grouped_task_agent': {
                        '$push': '$$ROOT'
                    }
                }
            }
            ]

            try:
                t_start = rospy.Time.now()
                results = self.coll_utils_results.aggregate(pipeline)
                t_end = (rospy.Time.now() - t_start).to_sec()
                rospy.loginfo("Computational time for dynamic risk" + str(t_end))
            except pymongo.errors.AutoReconnect:
                rospy.logerr(CONNECTION_LOST)
                return SetBoolResponse(False,NOT_SUCCESSFUL)
            
            rospy.loginfo(YELLOW + "Iterating results" + END)
            

            rospy.loginfo(GREEN + "Agents: {}".format(agents) + END)
            for task_group in results:          #A task_group contains vector of all (task_j, agent_i) same task computed by same agent
                input("New task group...")
                
                #Number of rows of regression matrix
                n_rows = len(task_group['grouped_task_agent'])
                
                # print(agents)
                
                #Compute other agents different from current task agent
                agent = task_group["_id"][1]                                    #First element of _id is name
                concurrent_agent = list(set(agents).difference(set([agent])))[0]         # If only one agent ok, otherwise there is also others agents in the set.
                
                rospy.loginfo("Principal agent: {}".format(agent))
                rospy.loginfo("Concurrent agent: {}".format(concurrent_agent))
                            
                        
                #for concurrent_agent in other_agents:                   #If more than one agents
                
                #Number of column of regression matrix = concurrent_agent task
                n_col = len(task_index[concurrent_agent])
                
                #Initialize regression matrix
                
                #regression_mat = np.zeros((n_rows,n_col))                       # To change: non è detto che per un task ce ne sia uno parallelo
                #row_vect = np.zeros((1,n_col))
                
                regression_mat = np.empty((0,n_col)) 
                known_vect = np.empty((0,1))
                
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
                    row_vect = np.zeros((1,n_col))
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
                    #Insert partial initial task
                    if p_initial_task:          # Not empty
                        if len(p_initial_task)>1:                                               #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
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
                                t_initial_mean = p_initial_task[0]["task_mean_informations"][0]["min"] #"task_mean_informations":[{"exp_dur":**}]
                                # print(task_index[concurrent_agent])
                                col_index_reg_mat = task_index[concurrent_agent].index(p_initial_task[0]["name"])
                                
                                #regression_mat[row, col_index_reg_mat] = delta_initial*t_initial_mean
                                
                                row_vect[0,col_index_reg_mat] += delta_initial*t_initial_mean
                                add_task_to_regmat = True
                                
                            self.updateConcurrentTaskCounters(p_initial_task[0]["name"], p_initial_task[0]["outcome"])
                            
                            #print(single_task)
                            #print(p_initial_task)
                            # print(p_initial_task[0]["name"])
                            # print(p_initial_task[0]["agent"])
                            # print(p_initial_task[0]["t_start"])
                            # print(p_initial_task[0]["t_end"])       
                            
                            # print(overlapping_initial_time)
                            # print(delta_initial)
                            # print(t_initial_mean)
                            # print(col_index_reg_mat)
                            
                    #Insert partial final task
                    if p_final_task:          # Not empty
                        if len(p_final_task)>1:
                            rospy.loginfo(RED + "More than one partial final task" + END)        #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                        else:
                            if p_final_task[0]["outcome"] == 1:          
                                rospy.loginfo(RED + "Final task: {}".format(p_final_task[0]["name"]) + END)

                                overlapping_final_time = single_task["t_end"] - p_final_task[0]["t_start"]
                                delta_final = overlapping_final_time / p_final_task[0]["delta_time"]
                                # print(p_final_task[0]["task_mean_informations"])
                                # print(p_final_task[0]["task_mean_informations"][0])
                                # print(p_final_task[0]["task_mean_informations"][0]["expected_duration"])
                                t_final_mean = p_final_task[0]["task_mean_informations"][0]["min"] #"task_mean_informations":[{"exp_dur":**}]
                                # print(concurrent_agent)
                                col_index_reg_mat = task_index[concurrent_agent].index(p_final_task[0]["name"])
                                
                                #regression_mat[row, col_index_reg_mat] = delta_final*t_final_mean
                                row_vect[0,col_index_reg_mat] += delta_final*t_final_mean
                                add_task_to_regmat = True

                            self.updateConcurrentTaskCounters(p_final_task[0]["name"], p_final_task[0]["outcome"])
                            #print(single_task)
                            
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
                            if inner_task["outcome"] == 1:               #QUesto [0] non sono sicuro
                                col_index_reg_mat = task_index[concurrent_agent].index(inner_task["name"])
                                # print("another inner tasks")
                                print(inner_task["name"])
                                # print(inner_task["task_mean_informations"])
                                # print(inner_task["task_mean_informations"][0]["expected_duration"])                        
                                # print(inner_task["task_mean_informations"][0])
                                #regression_mat[row, col_index_reg_mat] = inner_task["task_mean_informations"][0]["expected_duration"]
                                
                                row_vect[0,col_index_reg_mat] += inner_task["task_mean_informations"][0]["min"]
                                
                                overlapping_inner_tasks += inner_task["delta_time"]
                                add_task_to_regmat = True
                            
                            self.updateConcurrentTaskCounters(inner_task["name"], inner_task["outcome"])
                            # input("guarda inner task")
                            # print(inner_task["name"])
                    
                    if global_outer_task:
                        if len(global_outer_task)>1:
                            rospy.loginfo(RED + "More than one global outside final task" + END)        #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                        else:
                            if global_outer_task[0]["outcome"] == 1:          
                                rospy.loginfo(RED + "Global outside task: {}".format(global_outer_task[0]["name"]) + END)

                                delta_global_outside = single_task["delta_time"] / global_outer_task[0]["delta_time"]
                                
                                t_global_outside_task_mean = global_outer_task[0]["task_mean_informations"][0]["min"] #"task_mean_informations":[{"exp_dur":**}]
                                
                                col_index_reg_mat = task_index[concurrent_agent].index(global_outer_task[0]["name"])
                                #regression_mat[row, col_index_reg_mat] = delta_global_outside * t_global_outside_task_mean     # delta_time is the real duration of "inner task"

                                row_vect[0,col_index_reg_mat] += delta_global_outside * t_global_outside_task_mean

                                overlapping_global_outer_task = single_task["delta_time"]                                      # All the little task   |---|
                                add_task_to_regmat = True                                                                                                   # |--------|
                            self.updateConcurrentTaskCounters(global_outer_task[0]["name"], global_outer_task[0]["outcome"])
                    print(row_vect)
                    print(overlapping_initial_time)
                    print(overlapping_inner_tasks)
                    print(overlapping_final_time)
                    print(overlapping_global_outer_task)
                    if add_task_to_regmat:
                        #Nota TODO solo se almeno uno di quelli sopra
                        regression_mat = np.append(regression_mat, row_vect,axis=0)
                        # print("------------------")
                        # print(regression_mat)
                        # print("------------------")
                        
                        known_vect = np.append(known_vect, overlapping_initial_time + overlapping_final_time + overlapping_inner_tasks + overlapping_global_outer_task)
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
                
                t_start=rospy.Time.now()
                try:
                    lstsq_results=np.linalg.lstsq(regression_mat, known_vect, rcond=None)
                    print(lstsq_results)
                    dynamic_risk = lstsq_results[0]
                except np.linalg.LinAlgError:
                    rospy.loginfo(RED + "Least square does not converge" + END)
                    return SetBoolResponse(False,NOT_SUCCESSFUL)        
                
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
                        return SetBoolResponse(False,NOT_SUCCESSFUL)
                    
                rospy.loginfo(GREEN + "Fondamental task:" + END)
                print(single_task["name"])
                print(task_index[concurrent_agent])
                print(dynamic_risk)
                input("Leggi i risultati...")
                # for k in coefficient:
                #     input("Prossimo k")
                #     print(k)
                    
                print((rospy.Time.now()-t_start).to_sec())
                
            return SetBoolResponse(True,SUCCESSFUL)

    
        
    def dynamicRiskChart(self,request):
        """Method that is a callback of a service usefull for do dynamic risk heat map

        Args:
            request (SetBoolRequest): _description_

        Returns:
            SetBoolResponse: _description_
        """
        print("ciao")
        # Check if the results folder path not exist create it
        if not os.path.exists(self.results_folder_path):
            os.makedirs(self.results_folder_path)
            rospy.loginfo(RED + "The new direcory {} is created!".format(self.results_folder_path) + END)
        
        
        import pandas as pd
        import seaborn as sns

        pipeline_dr_grouped_by_agent = [        # Pipeline to group/separate dynamic risk elements by agent  
        {
            '$sort': {
                'agent_skill': 1, 
                'concurrent_skill': 1
            }
        }, {
            '$group': {
                '_id': '$agent', 
                'grouped_task_agent': {
                    '$push': '$$ROOT'
                }
            }
        }
        ]
        try:
            dynamic_risk_grouped = self.coll_interaction.aggregate(pipeline_dr_grouped_by_agent)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
        print("dopo")
        #### --------------- 
        dynamic_risk_list = []      # It will be a list of dictionaries each made like: {"main_agent":"name", "concurrent_agent":"name", "name_main_agent":[],  "name_concurrent_agent":[], "dynamic_risk":[]}
        print("dopo")

        for index, dynamic_risk_single_agent in enumerate(dynamic_risk_grouped):        # Iterate a group of dynamic_risk element (a group for main agent)
            dynamic_risk_list.append({})
            dynamic_risk_list[index][dynamic_risk_single_agent["_id"]] = []             # Main agent
            dynamic_risk_list[index]["dynamic_risk"] = []                               
            dynamic_risk_list[index]["main_agent"] = dynamic_risk_single_agent["_id"]   # In order to store who is main agent
            for dynamic_risk_single_element in dynamic_risk_single_agent["grouped_task_agent"]: 
                # print(dynamic_risk_single_element)
                
                # Add concurrent agent  
                if dynamic_risk_single_element["concurrent_agent"] not in dynamic_risk_list[index]:
                    dynamic_risk_list[index][dynamic_risk_single_element["concurrent_agent"]] = []                  
                    dynamic_risk_list[index]["concurrent_agent"] = dynamic_risk_single_element["concurrent_agent"]  # In order to store who is concurrent agent
                    #here dynamic_risk_list[index] = {"main_agent":"name", "concurrent_agent":"name", "name_main_agent":[],  "name_concurrent_agent":[], "dynamic_risk":[]}
                
                # Retrieve agent and concurrent agent skills 
                agent_skill = dynamic_risk_single_element["agent_skill"]
                concurrent_agent_skill = dynamic_risk_single_element["concurrent_skill"]
             
                #If pick_blue_box -> pick_blue_box_ + agent
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
        ##### -------------------
        
        # fig, axs = plt.subplots(1, 2)
        # fig.suptitle('Dynamic Matrix')
        
        for index, single_agent_dynamic_risk in enumerate(dynamic_risk_list):
            # Retrieve agent name
            main_agent = single_agent_dynamic_risk["main_agent"]
            concurrent_agent = single_agent_dynamic_risk["concurrent_agent"]
            
            # Remove it for chart data
            single_agent_dynamic_risk.pop("main_agent",None)
            single_agent_dynamic_risk.pop("concurrent_agent",None)
            
            # Create a dataframe
            data = pd.DataFrame(single_agent_dynamic_risk)
            print(data)
            print(True in data.duplicated())
            if data.duplicated().any():
                rospy.loginfo(RED + "There are duplicated task" + END)
                rospy.loginfo(data.duplicated())
                return SetBoolResponse(False,NOT_SUCCESSFUL)
            
            
            # Prepare data for heatmap
            data_matrix = data.pivot(main_agent,concurrent_agent,"dynamic_risk")

            plot_title = "Sinergy Matrix for agent: {}".format(main_agent)
            paper = True
            if paper:
                if main_agent=="ur5_on_guide":
                    main_agent_label = {"name":"Robot", "abbreviation":"R"}#("Robot","R")
                    concurrent_agent_label = {"name":"Human", "abbreviation":"H"}#("Robot","R")

                elif main_agent=="human_right_arm":
                    main_agent_label = {"name":"Human", "abbreviation":"H"}#("Human","H")
                    concurrent_agent_label = {"name":"Robot", "abbreviation":"R"}
                else:
                    main_agent_label = main_agent
                plot_title = "Sinergy Matrix for agent: {} ($S^{}$)".format(main_agent_label["name"],main_agent_label["abbreviation"])
                for agent in [main_agent,concurrent_agent]:
                    for k in data.index:
                        if data.at[k,agent] == "pick_blue_box_human_right_arm":
                            data.at[k,agent] = "Pick Blue Box (H)"
                        elif data.at[k,agent] == "place_blue_box_human_right_arm":
                            data.at[k,agent] = "Place Blue Box (H)"
                        elif data.at[k,agent] == "pick_blue_box_ur5_on_guide":
                            data.at[k,agent] = "Pick Blue Box (R)"
                        elif data.at[k,agent] == "place_blue_box_ur5_on_guide":
                            data.at[k,agent] = "Place Blue Box (R)"
                        elif data.at[k,agent] == "place_blue_box_ur5_on_guide":
                            data.at[k,agent] = "Place Blue Box"                       
                        elif data.at[k,agent] == "pick_white_box":
                            data.at[k,agent] = "Pick White Box"
                        elif data.at[k,agent] == "place_white_box":
                            data.at[k,agent] = "Place White Box"
                        elif data.at[k,agent] == "pick_orange_box":
                            data.at[k,agent] = "Pick Orange Box"
                        elif data.at[k,agent] == "place_orange_box":
                            data.at[k,agent] = "Place Orange Box"
                data = data.rename(columns={main_agent: main_agent_label["name"]+" Tasks", concurrent_agent: concurrent_agent_label["name"]+" Tasks"})
                print(data)
                data_matrix = data.pivot(main_agent_label["name"]+" Tasks",concurrent_agent_label["name"]+" Tasks","dynamic_risk")    
            else:
                main_agent_label = main_agent   #Remove label variable (unuseful)

            rospy.loginfo(RED + " ----------- "+ END)


            print(data_matrix)
            rospy.loginfo(RED + " ----------- "+ END)
            print(data_matrix)
            plt.figure(index)
            ax = sns.heatmap(data_matrix,annot=True)
            # plt.title(plot_title)
            ax.set_title(plot_title, pad=20)
        
            self.results_folder_path="/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_statistics/file/"
            plt.xticks(rotation=45, ha="right")
            plt.savefig(self.results_folder_path + "dynamic_risk_agent_" + main_agent +".png",bbox_inches='tight')
            plt.savefig(self.results_folder_path+"test"+"dynamic_risk_agent_"+".pdf",bbox_inches='tight')


        plt.show()            
        
        return SetBoolResponse(True,SUCCESSFUL)
                                                       
    def computeDynamicRiskNoAddInfo(self,request):     #senza medie nei concurrent
        t_start = rospy.Time.now()
        
        rospy.loginfo(SERVICE_CALLBACK.format(COMPUTE_DURATION_SERVICE))
        #Aggiunge delta_time, aggiunge la duration media del task "principale", + concurrent che sono senza info medie
        pipeline=[
        {
            '$addFields': {
                'delta_time': {
                    '$subtract': [
                        '$t_end', '$t_start'
                    ]
                }
            }
        }, {
            '$lookup': {
                'from': self.coll_durations_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$eq': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$name', '$$local_name'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'task_mean_informations'
            }
        }, {
            '$lookup': {
                'from': self.coll_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lte': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }, {
                                        '$gte': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'inner_task'
            }
        }, {
            '$lookup': {
                'from': self.coll_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_start', '$t_end'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }, {
                                        '$lt': [
                                             '$t_end','$$local_t_end'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'partial_task_initial'
            }
        }, {
            '$lookup': {
                'from': self.coll_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_end', '$t_start'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'partial_task_final'
            }
        }
        ]
        
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
            rospy.loginfo(RED + "Tempo impiegato: " + str((t_end-t_start).to_sec())+ END)

                #pprint.pprint(list(results))
            #rospy.loginfo(DURATION_OK)
            return SetBoolResponse(True,SUCCESSFUL)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False,NOT_SUCCESSFUL)        

    def drawTimeline(self,request):
        #Function to optimize for more than one agents and better graphics skills
        pipeline=[
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
                #print(task)
                recipe_task = []
                
                recipe_robot_task = []
                recipe_human_task = []
                rospy.loginfo(RED + "--------------------------------------------" + END)
                if "recipe_tasks" in task.keys():
                    fig, ax = plt.subplots()
                    for single_task in task["recipe_tasks"]:
                        #print(single_task)
                        #print((single_task['t_start'], single_task['delta_time'], single_task['agent']))
                        single_task['t_start']
                        if single_task['agent']=="ur5_on_guide":
                            recipe_robot_task.append((single_task['t_start'], single_task['delta_time']))
                        else:
                            recipe_human_task.append((single_task['t_start'], single_task['delta_time']))
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
                        recipe_task = list(map(lambda single_task : (single_task[0]-t_start_recipe,single_task[1]),recipe_task))
                        t_end_recipe = recipe_task[-1][0]+recipe_task[-1][1]
                        rospy.loginfo(RED + "T END" + END)
                        rospy.loginfo(t_end_recipe)
                        # rospy.loginfo(recipe_task)
                        rospy.loginfo(t_end_recipe)
                        recipe_robot_task = list(map(lambda single_task : (single_task[0]-t_start_recipe,single_task[1]),recipe_robot_task))
                        recipe_human_task = list(map(lambda single_task : (single_task[0]-t_start_recipe,single_task[1]),recipe_human_task))
                
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
                    ax.set_xlim(0, t_end_recipe*1.2)
                    # Setting graph attribute
                    ax.grid(True)
                    
                    colors_robot_task = plt.cm.BuPu(np.linspace(0, 0.5, len(recipe_robot_task)))
                    colors_human_task = plt.cm.BuPu(np.linspace(0, 0.5, len(recipe_human_task)))
                    
                    

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
                                facecolors=mcolors.TABLEAU_COLORS)
                    if recipe_human_task:
                        print("Human task")
                        print(recipe_human_task)
                        print(len(recipe_human_task))
                        ax.broken_barh(recipe_human_task,
                                (3, 2),                     
                                facecolors=mcolors.TABLEAU_COLORS)

                    ax.grid(True)
                    ax.set_title("Recipe: {}".format(n_recipe))
                    
                    if not os.path.exists(self.results_folder_path):
                        os.makedirs(self.results_folder_path)
                        rospy.loginfo(RED + "The new direcory {} is created!".format(self.results_folder_path) + END)
                    plt.savefig(self.results_folder_path + "recipe" + str(n_recipe) +".png")
                    rospy.loginfo(RED + "Recipe: {}".format(n_recipe) + END)
                    
                    # print(recipe_task)
                    # print("single task")
                    # print(task['recipe_tasks'])
                    input("Wait ..")
                    n_recipe+=1
    

            
            rospy.loginfo("Chart Results ok")
            return SetBoolResponse(True,SUCCESSFUL)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        pass

    def makeDurationChart(self,request):   #Compute dynamic risk with concurrent and mean information
        """Method 

        Args:
            request (SetBoolRequest): 

        Returns:
            SetBoolResponse: 
        """
        import pandas as pd
        import seaborn as sns
        
        if not self.createUtilsResults():
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        agents = self.getAgents()
        
        task_results = dict()           
        task_results["duration"] = []
        for agent in agents:
            task_results[agent] =[]
        
        # try:
        #     cursor_task_results= self.coll_durations.find({})
        # except pymongo.errors.AutoReconnect:
        #     rospy.logerr(CONNECTION_LOST)
        #     return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        # for cursor_result in cursor_task_results:

        pipeline = [
        {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lte': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }, {
                                        '$gte': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'inner_task'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_start', '$t_end'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$t_end', '$$local_t_end'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'partial_task_initial'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_end', '$t_start'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'partial_task_final'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'outer_task'
            }
        }, {
            '$group': {                     # Il group molto lento 0.1 -> 0.7
                '_id': [
                    '$name', '$agent'
                ], 
                'grouped_task_agent': {
                    '$push': '$$ROOT'
                }
            }
        }, {
            '$addFields': {
                'main_agent': {
                    '$arrayElemAt': [
                        '$_id', 1
                    ]
                }
            }
        }, {
            '$group': {
                '_id': '$main_agent', 
                'grouped_main_agent': {
                    '$push': '$$ROOT'
                }
            }
        }
        ]

        try:
            results = self.coll_utils_results.aggregate(pipeline)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        rospy.loginfo(YELLOW + "Iterating results" + END)
        

        rospy.loginfo(GREEN + "Agents: {}".format(agents) + END)
        for main_agent_group in results:
            main_agent = main_agent_group["_id"]
            concurrent_agent = list(set(agents).difference(set([main_agent])))[0]         # If only one agent ok, otherwise there is also others agents in the set.
            
            task_results_agent = copy.deepcopy(task_results)
            print(task_results_agent)
            input("verifica...")
            
            for task_group in main_agent_group["grouped_main_agent"]:          #A task_group contains vector of all (task_j, agent_i) same task computed by same agent
                # input("New task group...")
                
                #Number of rows of regression matrix
                
                #Compute other agents different from current task agent
                agent = task_group["_id"][1]                                    #First element of _id is name
                
                rospy.loginfo("Principal agent: {}".format(agent))
                rospy.loginfo("Concurrent agent: {}".format(concurrent_agent))

                rospy.loginfo(GREEN + "Fondamental task: {}".format(task_group["_id"][0]) + END)
                
                # task_results = dict()
                for single_task in task_group['grouped_task_agent']:

                    p_initial_task = single_task['partial_task_initial']
                    p_final_task = single_task['partial_task_final']
                    global_outer_task = single_task['outer_task']

                    rospy.loginfo(RED + "DURATION: {}".format(single_task["delta_time"]) + END)
                    #Insert partial initial task
                    if p_initial_task:          # Not empty
                        if len(p_initial_task)>1:                                               #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                            rospy.loginfo(RED + "More than one partial initial task" + END)
                        else:
                            if p_initial_task[0]["outcome"] == 1:
                                rospy.loginfo(RED + "Initial task: {}".format(p_initial_task[0]["name"]) + END)
                                print(task_results)
                                task_results_agent[agent].append(single_task["name"])
                                task_results_agent[concurrent_agent].append(p_initial_task[0]["name"])                    
                                task_results_agent["duration"].append(single_task["delta_time"])
                    #Insert partial final task
                    if p_final_task:          # Not empty
                        if len(p_final_task)>1:
                            rospy.loginfo(RED + "More than one partial final task" + END)        #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
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
                            if inner_task["outcome"] == 1:               #QUesto [0] non sono sicuro
                                print(inner_task["name"])
                        
                                task_results_agent[agent].append(single_task["name"])
                                task_results_agent[concurrent_agent].append(inner_task["name"])                    
                                task_results_agent["duration"].append(single_task["delta_time"])       
                                
                    if global_outer_task:
                        if len(global_outer_task)>1:
                            rospy.loginfo(RED + "More than one global outside final task" + END)        #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                        else:
                            if global_outer_task[0]["outcome"] == 1:          
                                rospy.loginfo(RED + "Global outside task: {}".format(global_outer_task[0]["name"]) + END)
                                task_results_agent[agent].append(single_task["name"])
                                task_results_agent[concurrent_agent].append(global_outer_task[0]["name"])                    
                                task_results_agent["duration"].append(single_task["delta_time"])                                   
                                
            print(task_results_agent)
                        
            dati = pd.DataFrame(task_results_agent)
            print(dati)
            sns.set_theme()
            # sns.catplot(data=dati, x=main_agent, y="duration", hue=concurrent_agent)
            sns.catplot(data=dati, kind="bar", x=main_agent, y="duration", hue=concurrent_agent)

            plt.figure()
            # sns.boxplot(data=dati, x=main_agent, y="duration", hue=concurrent_agent)
            

        plt.show()
        return SetBoolResponse(True,SUCCESSFUL)

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

    def computeDynamicRiskWithUncertainty(self,request):   #Compute dynamic risk with concurrent and mean information
        """Method for creating a collection of results with added the concurrent tasks

        Args:
            request (SetBoolRequest): _description_

        Returns:
            SetBoolResponse: _description_
        """
        import pandas as pd
        import statsmodels.api as sm
        import statsmodels.formula.api as smf

        t_start = rospy.Time.now()
        
        # Delete older dynamic risk collection
        try:
            self.coll_interaction.delete_many({})
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        # Create collection with results task + task mean information
        if not self.createUtilsResults():
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        # Pipeline for: retriving task name, unwinded for agents, each task only 1 agents
        pipeline_agents_task_name =[
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
            return SetBoolResponse(False,NOT_SUCCESSFUL)
                
        task_index = dict()                                                         # It will be as: {"agent_name":[list with all tasks that it can perform],..}
        for single_task in cursor_task_properties:
            if "name" in single_task.keys() and "agent" in single_task.keys() :     # Ensure it has nedded attributes
                if not single_task["name"] == "end":                                # Excule task end (not interesting)
                    if single_task["agent"] not in task_index:                      # If not exist already that agents
                        task_index[single_task["agent"]] = set()                    # A SET for each agent to ensure unique task
                    task_index[single_task["agent"]].add(single_task["name"])
                                    
        task_index = {key: list(values) for key, values in task_index.items()}       # Convert agents tasks from set to list (for have index)
        
        rospy.loginfo("Dizionario: agente->task list: ")
        rospy.loginfo(task_index)
        rospy.loginfo(RED + "OK FINO TASK RETRIEVE INFO" + END)
        # input("wait...")
        
        agents = list(task_index.keys())
        
        if len(agents)>2:
            rospy.loginfo(RED + "There are more than 2 agents in task properties" + END)
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        ############
        #Retrive results with their concurrent tasks information
        pipeline = [
        {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lte': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }, {
                                        '$gte': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'inner_task'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_start', '$t_end'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$t_end', '$$local_t_end'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'partial_task_initial'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_end', '$t_start'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'partial_task_final'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'outer_task'
            }
        }, {
            '$group': {                     # Il group molto lento 0.1 -> 0.7
                '_id': [
                    '$name', '$agent'
                ], 
                'grouped_task_agent': {
                    '$push': '$$ROOT'
                }
            }
        }
        ]

        try:
            t_start = rospy.Time.now()
            results = self.coll_utils_results.aggregate(pipeline)
            t_end = (rospy.Time.now() - t_start).to_sec()
            rospy.loginfo("Computational time for dynamic risk" + str(t_end))
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        rospy.loginfo(YELLOW + "Iterating results" + END)
        

        rospy.loginfo(GREEN + "Agents: {}".format(agents) + END)
        
        for task_group in results:          #A task_group contains vector of all (task_j, agent_i) same task computed by same agent
            input("New task group...")
            
            #Number of rows of regression matrix
            n_rows = len(task_group['grouped_task_agent'])
            
            # print(agents)
            
            #Compute other agents different from current task agent
            agent = task_group["_id"][1]                                    #First element of _id is name
            concurrent_agent = list(set(agents).difference(set([agent])))[0]         # If only one agent ok, otherwise there is also others agents in the set.
            
            rospy.loginfo("Principal agent: {}".format(agent))
            rospy.loginfo("Concurrent agent: {}".format(concurrent_agent))
                        
                       
            #for concurrent_agent in other_agents:                   #If more than one agents
            
            #Number of column of regression matrix = concurrent_agent task
            n_col = len(task_index[concurrent_agent])
            
            #Initialize regression matrix
            
            #regression_mat = np.zeros((n_rows,n_col))                       # To change: non è detto che per un task ce ne sia uno parallelo
            #row_vect = np.zeros((1,n_col))
            
            regression_mat = np.empty((0,n_col)) 
            known_vect = np.empty((0,1))
            
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
                row_vect = np.zeros((1,n_col))
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
                #Insert partial initial task
                if p_initial_task:          # Not empty
                    if len(p_initial_task)>1:                                               #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
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
                            t_initial_mean = p_initial_task[0]["task_mean_informations"][0]["expected_duration"] #"task_mean_informations":[{"exp_dur":**}]
                            # print(task_index[concurrent_agent])
                            col_index_reg_mat = task_index[concurrent_agent].index(p_initial_task[0]["name"])
                            
                            #regression_mat[row, col_index_reg_mat] = delta_initial*t_initial_mean
                            
                            row_vect[0,col_index_reg_mat] += delta_initial*t_initial_mean
                            add_task_to_regmat = True
                            
                        self.updateConcurrentTaskCounters(p_initial_task[0]["name"], p_initial_task[0]["outcome"])
                        
                        #print(single_task)
                        #print(p_initial_task)
                        # print(p_initial_task[0]["name"])
                        # print(p_initial_task[0]["agent"])
                        # print(p_initial_task[0]["t_start"])
                        # print(p_initial_task[0]["t_end"])       
                        
                        # print(overlapping_initial_time)
                        # print(delta_initial)
                        # print(t_initial_mean)
                        # print(col_index_reg_mat)
                        
                #Insert partial final task
                if p_final_task:          # Not empty
                    if len(p_final_task)>1:
                        rospy.loginfo(RED + "More than one partial final task" + END)        #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                    else:
                        if p_final_task[0]["outcome"] == 1:          
                            rospy.loginfo(RED + "Final task: {}".format(p_final_task[0]["name"]) + END)

                            overlapping_final_time = single_task["t_end"] - p_final_task[0]["t_start"]
                            delta_final = overlapping_final_time / p_final_task[0]["delta_time"]
                            # print(p_final_task[0]["task_mean_informations"])
                            # print(p_final_task[0]["task_mean_informations"][0])
                            # print(p_final_task[0]["task_mean_informations"][0]["expected_duration"])
                            t_final_mean = p_final_task[0]["task_mean_informations"][0]["expected_duration"] #"task_mean_informations":[{"exp_dur":**}]
                            # print(concurrent_agent)
                            col_index_reg_mat = task_index[concurrent_agent].index(p_final_task[0]["name"])
                            
                            #regression_mat[row, col_index_reg_mat] = delta_final*t_final_mean
                            row_vect[0,col_index_reg_mat] += delta_final*t_final_mean
                            add_task_to_regmat = True

                        self.updateConcurrentTaskCounters(p_final_task[0]["name"], p_final_task[0]["outcome"])
                        #print(single_task)
                        
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
                        if inner_task["outcome"] == 1:               #QUesto [0] non sono sicuro
                            col_index_reg_mat = task_index[concurrent_agent].index(inner_task["name"])
                            # print("another inner tasks")
                            print(inner_task["name"])
                            # print(inner_task["task_mean_informations"])
                            # print(inner_task["task_mean_informations"][0]["expected_duration"])                        
                            # print(inner_task["task_mean_informations"][0])
                            #regression_mat[row, col_index_reg_mat] = inner_task["task_mean_informations"][0]["expected_duration"]
                            
                            row_vect[0,col_index_reg_mat] += inner_task["task_mean_informations"][0]["expected_duration"]
                            
                            overlapping_inner_tasks += inner_task["delta_time"]
                            add_task_to_regmat = True
                        
                        self.updateConcurrentTaskCounters(inner_task["name"], inner_task["outcome"])
                        # input("guarda inner task")
                        # print(inner_task["name"])
                
                if global_outer_task:
                    if len(global_outer_task)>1:
                        rospy.loginfo(RED + "More than one global outside final task" + END)        #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                    else:
                        if global_outer_task[0]["outcome"] == 1:          
                            rospy.loginfo(RED + "Global outside task: {}".format(global_outer_task[0]["name"]) + END)

                            delta_global_outside = single_task["delta_time"] / global_outer_task[0]["delta_time"]
                            
                            t_global_outside_task_mean = global_outer_task[0]["task_mean_informations"][0]["expected_duration"] #"task_mean_informations":[{"exp_dur":**}]
                            
                            col_index_reg_mat = task_index[concurrent_agent].index(global_outer_task[0]["name"])
                            #regression_mat[row, col_index_reg_mat] = delta_global_outside * t_global_outside_task_mean     # delta_time is the real duration of "inner task"

                            row_vect[0,col_index_reg_mat] += delta_global_outside * t_global_outside_task_mean

                            overlapping_global_outer_task = single_task["delta_time"]                                      # All the little task   |---|
                            add_task_to_regmat = True                                                                                                   # |--------|
                        self.updateConcurrentTaskCounters(global_outer_task[0]["name"], global_outer_task[0]["outcome"])
                print(row_vect)
                print(overlapping_initial_time)
                print(overlapping_inner_tasks)
                print(overlapping_final_time)
                print(overlapping_global_outer_task)
                if add_task_to_regmat:
                    #Nota TODO solo se almeno uno di quelli sopra
                    regression_mat = np.append(regression_mat, row_vect,axis=0)
                    # print("------------------")
                    # print(regression_mat)
                    # print("------------------")
                    
                    known_vect = np.append(known_vect, overlapping_initial_time + overlapping_final_time + overlapping_inner_tasks + overlapping_global_outer_task)
                    # print(known_vect)
                    # print("------------------")
                    # known_vect[row] = overlapping_initial_time + overlapping_final_time + overlapping_inner_tasks + overlapping_global_outer_task  # = single_task["delta_time"]-t_idle : t_idle = single_task["delta_time"] - overlapping_inner -overl_initial-over_final 
                # print(known_vect)
                print(known_vect)
                # input("COntrolla")
            #input("Regression...")
            print(regression_mat)
            print(known_vect)
            # print(regression_mat.shape)
            # print(known_vect.shape)
            
            t_start=rospy.Time.now()
            try:
                lstsq_results=np.linalg.lstsq(regression_mat, known_vect, rcond=None)
                print(lstsq_results)
                dynamic_risk = lstsq_results[0]
            except np.linalg.LinAlgError:
                rospy.loginfo(RED + "Least square does not converge" + END)
                return SetBoolResponse(False,NOT_SUCCESSFUL)        
            
            print("**************")
            (row,col) = regression_mat.shape
            dizionario = dict()
            formula_string = ""
            for index in range(0,col):
                dizionario[task_index[concurrent_agent][index]] = regression_mat[:,index]
                formula_string +=task_index[concurrent_agent][index]
                if index < col - 1:
                    formula_string += " + "
            rospy.loginfo(formula_string)     
            input("Guarda  formula")   
            dizionario["duration"] = known_vect
            print(dizionario)    
            dataF = pd.DataFrame(dizionario)
            
            reg_model = smf.ols(formula="duration ~ " + formula_string + " -1 ", data=dataF)
            reg_result = reg_model.fit()
            print(reg_result.summary())
            
            input("stronzo")
            print("**************")
            
            #Here agent and concurrent_agent change role for dynamic risk 
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
                    return SetBoolResponse(False,NOT_SUCCESSFUL)
                
            rospy.loginfo(GREEN + "Fondamental task:" + END)
            print(single_task["name"])
            print(task_index[concurrent_agent])
            print(dynamic_risk)
            input("Leggi i risultati...")
            # for k in coefficient:
            #     input("Prossimo k")
            #     print(k)
                
            print((rospy.Time.now()-t_start).to_sec())
            
        return SetBoolResponse(True,SUCCESSFUL)
    def computeDynamicRiskWithUncertaintyMin(self,request):   #Compute dynamic risk with concurrent and mean information
        """Method for creating a collection of results with added the concurrent tasks

        Args:
            request (SetBoolRequest): _description_

        Returns:
            SetBoolResponse: _description_
        """
        import pandas as pd
        import statsmodels.api as sm
        import statsmodels.formula.api as smf

        t_start = rospy.Time.now()
        
        # Delete older dynamic risk collection
        try:
            self.coll_interaction.delete_many({})
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        # Create collection with results task + task mean information
        if not self.createUtilsResults():
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        # Pipeline for: retriving task name, unwinded for agents, each task only 1 agents
        pipeline_agents_task_name =[
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
            return SetBoolResponse(False,NOT_SUCCESSFUL)
                
        task_index = dict()                                                         # It will be as: {"agent_name":[list with all tasks that it can perform],..}
        for single_task in cursor_task_properties:
            if "name" in single_task.keys() and "agent" in single_task.keys() :     # Ensure it has nedded attributes
                if not single_task["name"] == "end":                                # Excule task end (not interesting)
                    if single_task["agent"] not in task_index:                      # If not exist already that agents
                        task_index[single_task["agent"]] = set()                    # A SET for each agent to ensure unique task
                    task_index[single_task["agent"]].add(single_task["name"])
                                    
        task_index = {key: list(values) for key, values in task_index.items()}       # Convert agents tasks from set to list (for have index)
        
        rospy.loginfo("Dizionario: agente->task list: ")
        rospy.loginfo(task_index)
        rospy.loginfo(RED + "OK FINO TASK RETRIEVE INFO" + END)
        # input("wait...")
        
        agents = list(task_index.keys())
        
        if len(agents)>2:
            rospy.loginfo(RED + "There are more than 2 agents in task properties" + END)
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        ############
        #Retrive results with their concurrent tasks information
        pipeline = [
        {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lte': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }, {
                                        '$gte': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'inner_task'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_start', '$t_end'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$t_end', '$$local_t_end'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'partial_task_initial'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_end', '$t_start'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'partial_task_final'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'outer_task'
            }
        }, {
            '$group': {                     # Il group molto lento 0.1 -> 0.7
                '_id': [
                    '$name', '$agent'
                ], 
                'grouped_task_agent': {
                    '$push': '$$ROOT'
                }
            }
        }
        ]

        try:
            t_start = rospy.Time.now()
            results = self.coll_utils_results.aggregate(pipeline)
            t_end = (rospy.Time.now() - t_start).to_sec()
            rospy.loginfo("Computational time for dynamic risk" + str(t_end))
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        rospy.loginfo(YELLOW + "Iterating results" + END)
        

        rospy.loginfo(GREEN + "Agents: {}".format(agents) + END)
        for task_group in results:          #A task_group contains vector of all (task_j, agent_i) same task computed by same agent
            input("New task group...")
            
            #Number of rows of regression matrix
            n_rows = len(task_group['grouped_task_agent'])
            
            # print(agents)
            
            #Compute other agents different from current task agent
            agent = task_group["_id"][1]                                    #First element of _id is name
            concurrent_agent = list(set(agents).difference(set([agent])))[0]         # If only one agent ok, otherwise there is also others agents in the set.
            
            rospy.loginfo("Principal agent: {}".format(agent))
            rospy.loginfo("Concurrent agent: {}".format(concurrent_agent))
                        
                       
            #for concurrent_agent in other_agents:                   #If more than one agents
            
            #Number of column of regression matrix = concurrent_agent task
            n_col = len(task_index[concurrent_agent])
            
            #Initialize regression matrix
            
            #regression_mat = np.zeros((n_rows,n_col))                       # To change: non è detto che per un task ce ne sia uno parallelo
            #row_vect = np.zeros((1,n_col))
            
            regression_mat = np.empty((0,n_col)) 
            known_vect = np.empty((0,1))
            
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
                row_vect = np.zeros((1,n_col))
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
                #Insert partial initial task
                if p_initial_task:          # Not empty
                    if len(p_initial_task)>1:                                               #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
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
                            t_initial_mean = p_initial_task[0]["task_mean_informations"][0]["min"] #"task_mean_informations":[{"exp_dur":**}]
                            # print(task_index[concurrent_agent])
                            col_index_reg_mat = task_index[concurrent_agent].index(p_initial_task[0]["name"])
                            
                            #regression_mat[row, col_index_reg_mat] = delta_initial*t_initial_mean
                            
                            row_vect[0,col_index_reg_mat] += delta_initial*t_initial_mean
                            add_task_to_regmat = True
                            
                        self.updateConcurrentTaskCounters(p_initial_task[0]["name"], p_initial_task[0]["outcome"])
                        
                        #print(single_task)
                        #print(p_initial_task)
                        # print(p_initial_task[0]["name"])
                        # print(p_initial_task[0]["agent"])
                        # print(p_initial_task[0]["t_start"])
                        # print(p_initial_task[0]["t_end"])       
                        
                        # print(overlapping_initial_time)
                        # print(delta_initial)
                        # print(t_initial_mean)
                        # print(col_index_reg_mat)
                        
                #Insert partial final task
                if p_final_task:          # Not empty
                    if len(p_final_task)>1:
                        rospy.loginfo(RED + "More than one partial final task" + END)        #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                    else:
                        if p_final_task[0]["outcome"] == 1:          
                            rospy.loginfo(RED + "Final task: {}".format(p_final_task[0]["name"]) + END)

                            overlapping_final_time = single_task["t_end"] - p_final_task[0]["t_start"]
                            delta_final = overlapping_final_time / p_final_task[0]["delta_time"]
                            # print(p_final_task[0]["task_mean_informations"])
                            # print(p_final_task[0]["task_mean_informations"][0])
                            # print(p_final_task[0]["task_mean_informations"][0]["expected_duration"])
                            t_final_mean = p_final_task[0]["task_mean_informations"][0]["min"] #"task_mean_informations":[{"exp_dur":**}]
                            # print(concurrent_agent)
                            col_index_reg_mat = task_index[concurrent_agent].index(p_final_task[0]["name"])
                            
                            #regression_mat[row, col_index_reg_mat] = delta_final*t_final_mean
                            row_vect[0,col_index_reg_mat] += delta_final*t_final_mean
                            add_task_to_regmat = True

                        self.updateConcurrentTaskCounters(p_final_task[0]["name"], p_final_task[0]["outcome"])
                        #print(single_task)
                        
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
                        if inner_task["outcome"] == 1:               #QUesto [0] non sono sicuro
                            col_index_reg_mat = task_index[concurrent_agent].index(inner_task["name"])
                            # print("another inner tasks")
                            print(inner_task["name"])
                            # print(inner_task["task_mean_informations"])
                            # print(inner_task["task_mean_informations"][0]["expected_duration"])                        
                            # print(inner_task["task_mean_informations"][0])
                            #regression_mat[row, col_index_reg_mat] = inner_task["task_mean_informations"][0]["expected_duration"]
                            
                            row_vect[0,col_index_reg_mat] += inner_task["task_mean_informations"][0]["min"]
                            
                            overlapping_inner_tasks += inner_task["delta_time"]
                            add_task_to_regmat = True
                        
                        self.updateConcurrentTaskCounters(inner_task["name"], inner_task["outcome"])
                        # input("guarda inner task")
                        # print(inner_task["name"])
                
                if global_outer_task:
                    if len(global_outer_task)>1:
                        rospy.loginfo(RED + "More than one global outside final task" + END)        #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                    else:
                        if global_outer_task[0]["outcome"] == 1:          
                            rospy.loginfo(RED + "Global outside task: {}".format(global_outer_task[0]["name"]) + END)

                            delta_global_outside = single_task["delta_time"] / global_outer_task[0]["delta_time"]
                            
                            t_global_outside_task_mean = global_outer_task[0]["task_mean_informations"][0]["min"] #"task_mean_informations":[{"exp_dur":**}]
                            
                            col_index_reg_mat = task_index[concurrent_agent].index(global_outer_task[0]["name"])
                            #regression_mat[row, col_index_reg_mat] = delta_global_outside * t_global_outside_task_mean     # delta_time is the real duration of "inner task"

                            row_vect[0,col_index_reg_mat] += delta_global_outside * t_global_outside_task_mean

                            overlapping_global_outer_task = single_task["delta_time"]                                      # All the little task   |---|
                            add_task_to_regmat = True                                                                                                   # |--------|
                        self.updateConcurrentTaskCounters(global_outer_task[0]["name"], global_outer_task[0]["outcome"])
                print(row_vect)
                print(overlapping_initial_time)
                print(overlapping_inner_tasks)
                print(overlapping_final_time)
                print(overlapping_global_outer_task)
                if add_task_to_regmat:
                    #Nota TODO solo se almeno uno di quelli sopra
                    regression_mat = np.append(regression_mat, row_vect,axis=0)
                    # print("------------------")
                    # print(regression_mat)
                    # print("------------------")
                    
                    known_vect = np.append(known_vect, overlapping_initial_time + overlapping_final_time + overlapping_inner_tasks + overlapping_global_outer_task)
                    # print(known_vect)
                    # print("------------------")
                    # known_vect[row] = overlapping_initial_time + overlapping_final_time + overlapping_inner_tasks + overlapping_global_outer_task  # = single_task["delta_time"]-t_idle : t_idle = single_task["delta_time"] - overlapping_inner -overl_initial-over_final 
                # print(known_vect)
                print(known_vect)
                # input("COntrolla")
            #input("Regression...")
            print(regression_mat)
            print(known_vect)
            # print(regression_mat.shape)
            # print(known_vect.shape)
            
            t_start=rospy.Time.now()
            try:
                lstsq_results=np.linalg.lstsq(regression_mat, known_vect, rcond=None)
                print(lstsq_results)
                dynamic_risk = lstsq_results[0]
            except np.linalg.LinAlgError:
                rospy.loginfo(RED + "Least square does not converge" + END)
                return SetBoolResponse(False,NOT_SUCCESSFUL)        
            
            print("**************")
            (row,col) = regression_mat.shape
            dizionario = dict()
            formula_string = ""
            for index in range(0,col):
                dizionario[task_index[concurrent_agent][index]] = regression_mat[:,index]
                formula_string +=task_index[concurrent_agent][index]
                if index < col - 1:
                    formula_string += " + "
            rospy.loginfo(formula_string)     
            input("Guarda  formula")   
            dizionario["duration"] = known_vect
            print(dizionario)    
            dataF = pd.DataFrame(dizionario)
            
            reg_model = smf.ols(formula="duration ~ " + formula_string + " -1 ", data=dataF)
            reg_result = reg_model.fit()
            
            print(reg_result.summary())
            
            input("stronzo")
            print("**************")
            
            #Here agent and concurrent_agent change role for dynamic risk 
            for index, task in enumerate(task_index[concurrent_agent]):
                counter, success_rate = self.getConcurrentTaskStatistics(task)
                print(task)
                print(reg_result.bse[task])
                try:
                    results = self.coll_utils_results.aggregate(pipeline)
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
                    return SetBoolResponse(False,NOT_SUCCESSFUL)
                
            rospy.loginfo(GREEN + "Fondamental task:" + END)
            print(single_task["name"])
            print(task_index[concurrent_agent])
            print(dynamic_risk)
            input("Leggi i risultati...")
            # for k in coefficient:
            #     input("Prossimo k")
            #     print(k)
                
            print((rospy.Time.now()-t_start).to_sec())
            
        return SetBoolResponse(True,SUCCESSFUL)

    def computeDynamicRiskWithUncertaintyMedian(self,request):   #Compute dynamic risk with concurrent and mean information
        """Method for creating a collection of results with added the concurrent tasks

        Args:
            request (SetBoolRequest): _description_

        Returns:
            SetBoolResponse: _description_
        """
        import pandas as pd
        import statsmodels.api as sm
        import statsmodels.formula.api as smf

        t_start = rospy.Time.now()
        
        # Delete older dynamic risk collection
        try:
            self.coll_interaction.delete_many({})
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        # Create collection with results task + task mean information
        
        # Pipeline for: retriving task name, unwinded for agents, each task only 1 agents
        pipeline_agents_task_name =[
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
            return SetBoolResponse(False,NOT_SUCCESSFUL)
                
        task_index = dict()                                                         # It will be as: {"agent_name":[list with all tasks that it can perform],..}
        for single_task in cursor_task_properties:
            if "name" in single_task.keys() and "agent" in single_task.keys() :     # Ensure it has nedded attributes
                if not single_task["name"] == "end":                                # Excule task end (not interesting)
                    if single_task["agent"] not in task_index:                      # If not exist already that agents
                        task_index[single_task["agent"]] = set()                    # A SET for each agent to ensure unique task
                    task_index[single_task["agent"]].add(single_task["name"])
                                    
        task_index = {key: list(values) for key, values in task_index.items()}       # Convert agents tasks from set to list (for have index)
        
        rospy.loginfo("Dizionario: agente->task list: ")
        rospy.loginfo(task_index)
        rospy.loginfo(RED + "OK FINO TASK RETRIEVE INFO" + END)
        
        pipeline_for_median=[
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
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        for task_group in cursor_task_median:
            agent = task_group["_id"]["agent"]
            task_name = task_group["_id"]["name"]
            
            task_duration_vector = np.empty((len(task_group["all"])))

            for id,task in enumerate(task_group["all"]): 
                print(task)
                task_duration_vector[id]=task["t_end"]-task["t_start"]
            median=np.median(task_duration_vector)
            try:
                self.coll_durations.update_one({"_id.agent":agent,"_id.name":task_name},{"$set": {"median": median}})

            except pymongo.errors.AutoReconnect:
                rospy.logerr(CONNECTION_LOST)
                return SetBoolResponse(False,NOT_SUCCESSFUL)

        if not self.createUtilsResults():
            return SetBoolResponse(False,NOT_SUCCESSFUL)
            
        # input("wait...")
        
        agents = list(task_index.keys())
        
        if len(agents)>2:
            rospy.loginfo(RED + "There are more than 2 agents in task properties" + END)
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        ############
        #Retrive results with their concurrent tasks information
        pipeline = [
        {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lte': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }, {
                                        '$gte': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'inner_task'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_start', '$t_end'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$t_end', '$$local_t_end'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'partial_task_initial'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_end', '$t_start'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'partial_task_final'
            }
        }, {
            '$lookup': {
                'from': self.utils_results_name, 
                'let': {
                    'local_name': '$name', 
                    'local_agent': '$agent', 
                    'local_recipe': '$recipe', 
                    'local_t_start': '$t_start', 
                    'local_t_end': '$t_end'
                }, 
                'pipeline': [
                    {
                        '$match': {
                            '$expr': {
                                '$and': [
                                    {
                                        '$ne': [
                                            '$agent', '$$local_agent'
                                        ]
                                    }, {
                                        '$eq': [
                                            '$recipe', '$$local_recipe'
                                        ]
                                    }, {
                                        '$lt': [
                                            '$$local_t_end', '$t_end'
                                        ]
                                    }, {
                                        '$gt': [
                                            '$$local_t_start', '$t_start'
                                        ]
                                    }
                                ]
                            }
                        }
                    }, {
                        '$project': {
                            'stock_item': 0, 
                            '_id': 0
                        }
                    }
                ], 
                'as': 'outer_task'
            }
        }, {
            '$group': {                     # Il group molto lento 0.1 -> 0.7
                '_id': [
                    '$name', '$agent'
                ], 
                'grouped_task_agent': {
                    '$push': '$$ROOT'
                }
            }
        }
        ]

        try:
            t_start = rospy.Time.now()
            results = self.coll_utils_results.aggregate(pipeline)
            t_end = (rospy.Time.now() - t_start).to_sec()
            rospy.loginfo("Computational time for dynamic risk" + str(t_end))
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        rospy.loginfo(YELLOW + "Iterating results" + END)
        

        rospy.loginfo(GREEN + "Agents: {}".format(agents) + END)
        for task_group in results:          #A task_group contains vector of all (task_j, agent_i) same task computed by same agent
            input("New task group...")
            
            #Number of rows of regression matrix
            n_rows = len(task_group['grouped_task_agent'])
            
            # print(agents)
            
            #Compute other agents different from current task agent
            agent = task_group["_id"][1]                                    #First element of _id is name
            concurrent_agent = list(set(agents).difference(set([agent])))[0]         # If only one agent ok, otherwise there is also others agents in the set.
            
            rospy.loginfo("Principal agent: {}".format(agent))
            rospy.loginfo("Concurrent agent: {}".format(concurrent_agent))
                        
                       
            #for concurrent_agent in other_agents:                   #If more than one agents
            
            #Number of column of regression matrix = concurrent_agent task
            n_col = len(task_index[concurrent_agent])
            
            #Initialize regression matrix
            
            #regression_mat = np.zeros((n_rows,n_col))                       # To change: non è detto che per un task ce ne sia uno parallelo
            #row_vect = np.zeros((1,n_col))
            
            regression_mat = np.empty((0,n_col)) 
            known_vect = np.empty((0,1))
            
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
                row_vect = np.zeros((1,n_col))
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
                #Insert partial initial task
                if p_initial_task:          # Not empty
                    if len(p_initial_task)>1:                                               #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
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
                            t_initial_mean = p_initial_task[0]["task_mean_informations"][0]["median"] #"task_mean_informations":[{"exp_dur":**}]
                            # print(task_index[concurrent_agent])
                            col_index_reg_mat = task_index[concurrent_agent].index(p_initial_task[0]["name"])
                            
                            #regression_mat[row, col_index_reg_mat] = delta_initial*t_initial_mean
                            
                            row_vect[0,col_index_reg_mat] += delta_initial*t_initial_mean
                            add_task_to_regmat = True
                            
                        self.updateConcurrentTaskCounters(p_initial_task[0]["name"], p_initial_task[0]["outcome"])
                        
                        #print(single_task)
                        #print(p_initial_task)
                        # print(p_initial_task[0]["name"])
                        # print(p_initial_task[0]["agent"])
                        # print(p_initial_task[0]["t_start"])
                        # print(p_initial_task[0]["t_end"])       
                        
                        # print(overlapping_initial_time)
                        # print(delta_initial)
                        # print(t_initial_mean)
                        # print(col_index_reg_mat)
                        
                #Insert partial final task
                if p_final_task:          # Not empty
                    if len(p_final_task)>1:
                        rospy.loginfo(RED + "More than one partial final task" + END)        #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                    else:
                        if p_final_task[0]["outcome"] == 1:          
                            rospy.loginfo(RED + "Final task: {}".format(p_final_task[0]["name"]) + END)

                            overlapping_final_time = single_task["t_end"] - p_final_task[0]["t_start"]
                            delta_final = overlapping_final_time / p_final_task[0]["delta_time"]
                            # print(p_final_task[0]["task_mean_informations"])
                            # print(p_final_task[0]["task_mean_informations"][0])
                            # print(p_final_task[0]["task_mean_informations"][0]["expected_duration"])
                            t_final_mean = p_final_task[0]["task_mean_informations"][0]["median"] #"task_mean_informations":[{"exp_dur":**}]
                            # print(concurrent_agent)
                            col_index_reg_mat = task_index[concurrent_agent].index(p_final_task[0]["name"])
                            
                            #regression_mat[row, col_index_reg_mat] = delta_final*t_final_mean
                            row_vect[0,col_index_reg_mat] += delta_final*t_final_mean
                            add_task_to_regmat = True

                        self.updateConcurrentTaskCounters(p_final_task[0]["name"], p_final_task[0]["outcome"])
                        #print(single_task)
                        
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
                        if inner_task["outcome"] == 1:               #QUesto [0] non sono sicuro
                            col_index_reg_mat = task_index[concurrent_agent].index(inner_task["name"])
                            # print("another inner tasks")
                            print(inner_task["name"])
                            # print(inner_task["task_mean_informations"])
                            # print(inner_task["task_mean_informations"][0]["expected_duration"])                        
                            # print(inner_task["task_mean_informations"][0])
                            #regression_mat[row, col_index_reg_mat] = inner_task["task_mean_informations"][0]["expected_duration"]
                            
                            row_vect[0,col_index_reg_mat] += inner_task["task_mean_informations"][0]["median"]
                            
                            overlapping_inner_tasks += inner_task["delta_time"]
                            add_task_to_regmat = True
                        
                        self.updateConcurrentTaskCounters(inner_task["name"], inner_task["outcome"])
                        # input("guarda inner task")
                        # print(inner_task["name"])
                
                if global_outer_task:
                    if len(global_outer_task)>1:
                        rospy.loginfo(RED + "More than one global outside final task" + END)        #Future note: It can be more than one agents in parallel, in that case filter to consider only concurrent task of concurrent agent
                    else:
                        if global_outer_task[0]["outcome"] == 1:          
                            rospy.loginfo(RED + "Global outside task: {}".format(global_outer_task[0]["name"]) + END)

                            delta_global_outside = single_task["delta_time"] / global_outer_task[0]["delta_time"]
                            
                            t_global_outside_task_mean = global_outer_task[0]["task_mean_informations"][0]["median"] #"task_mean_informations":[{"exp_dur":**}]
                            
                            col_index_reg_mat = task_index[concurrent_agent].index(global_outer_task[0]["name"])
                            #regression_mat[row, col_index_reg_mat] = delta_global_outside * t_global_outside_task_mean     # delta_time is the real duration of "inner task"

                            row_vect[0,col_index_reg_mat] += delta_global_outside * t_global_outside_task_mean

                            overlapping_global_outer_task = single_task["delta_time"]                                      # All the little task   |---|
                            add_task_to_regmat = True                                                                                                   # |--------|
                        self.updateConcurrentTaskCounters(global_outer_task[0]["name"], global_outer_task[0]["outcome"])
                print(row_vect)
                print(overlapping_initial_time)
                print(overlapping_inner_tasks)
                print(overlapping_final_time)
                print(overlapping_global_outer_task)
                if add_task_to_regmat:
                    #Nota TODO solo se almeno uno di quelli sopra
                    regression_mat = np.append(regression_mat, row_vect,axis=0)
                    # print("------------------")
                    # print(regression_mat)
                    # print("------------------")
                    
                    known_vect = np.append(known_vect, overlapping_initial_time + overlapping_final_time + overlapping_inner_tasks + overlapping_global_outer_task)
                    # print(known_vect)
                    # print("------------------")
                    # known_vect[row] = overlapping_initial_time + overlapping_final_time + overlapping_inner_tasks + overlapping_global_outer_task  # = single_task["delta_time"]-t_idle : t_idle = single_task["delta_time"] - overlapping_inner -overl_initial-over_final 
                # print(known_vect)
                print(known_vect)
                # input("COntrolla")
            #input("Regression...")
            print(regression_mat)
            print(known_vect)
            # print(regression_mat.shape)
            # print(known_vect.shape)
            
            t_start=rospy.Time.now()
            try:
                lstsq_results=np.linalg.lstsq(regression_mat, known_vect, rcond=None)
                print(lstsq_results)
                dynamic_risk = lstsq_results[0]
            except np.linalg.LinAlgError:
                rospy.loginfo(RED + "Least square does not converge" + END)
                return SetBoolResponse(False,NOT_SUCCESSFUL)        
            
            print("**************")
            (row,col) = regression_mat.shape
            dizionario = dict()
            formula_string = ""
            for index in range(0,col):
                dizionario[task_index[concurrent_agent][index]] = regression_mat[:,index]
                formula_string +=task_index[concurrent_agent][index]
                if index < col - 1:
                    formula_string += " + "
            rospy.loginfo(formula_string)     
            input("Guarda  formula")   
            dizionario["duration"] = known_vect
            print(dizionario)    
            dataF = pd.DataFrame(dizionario)
            
            reg_model = smf.ols(formula="duration ~ " + formula_string + " -1 ", data=dataF)
            reg_result = reg_model.fit()
            
            print(reg_result.summary())
            
            input("stronzo")
            print("**************")
            
            #Here agent and concurrent_agent change role for dynamic risk 
            for index, task in enumerate(task_index[concurrent_agent]):
                counter, success_rate = self.getConcurrentTaskStatistics(task)
                print(task)
                print(reg_result.bse[task])
                try:
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
                    return SetBoolResponse(False,NOT_SUCCESSFUL)
                
            rospy.loginfo(GREEN + "Fondamental task:" + END)
            print(single_task["name"])
            print(task_index[concurrent_agent])
            print(dynamic_risk)
            input("Leggi i risultati...")
            # for k in coefficient:
            #     input("Prossimo k")
            #     print(k)
                
            print((rospy.Time.now()-t_start).to_sec())
            
        return SetBoolResponse(True,SUCCESSFUL)

    def dynamicRiskUncertaintyChart(self,request):
        import pandas as pd
        import seaborn as sns

        pipeline_dr_grouped_by_agent = [        # Pipeline to group/separate dynamic risk elements by agent  
        {
            '$sort': {
                'agent_skill': 1, 
                'concurrent_skill': 1
            }
        }, {
            '$group': {
                '_id': '$agent', 
                'grouped_task_agent': {
                    '$push': '$$ROOT'
                }
            }
        }
        ]
        try:
            dynamic_risk_grouped = self.coll_interaction.aggregate(pipeline_dr_grouped_by_agent)
        except pymongo.errors.AutoReconnect:
            rospy.logerr("Connectrion lost")
            return SetBoolResponse(False,NOT_SUCCESSFUL)
        
        dynamic_risk_list = []
        for index, dynamic_risk_single_agent in enumerate(dynamic_risk_grouped):        # Iterate a group of dynamic_risk element (a group for main agent)
            dynamic_risk_list.append({})
            dynamic_risk_list[index][dynamic_risk_single_agent["_id"]] = []             # Main agent
            dynamic_risk_list[index]["dynamic_risk"] = []  
            dynamic_risk_list[index]["std_err"] = []                             
            dynamic_risk_list[index]["main_agent"] = dynamic_risk_single_agent["_id"]   # In order to store who is main agent
            for dynamic_risk_single_element in dynamic_risk_single_agent["grouped_task_agent"]: 
                
                # Add concurrent agent  
                if dynamic_risk_single_element["concurrent_agent"] not in dynamic_risk_list[index]:
                    dynamic_risk_list[index][dynamic_risk_single_element["concurrent_agent"]] = []                  
                    dynamic_risk_list[index]["concurrent_agent"] = dynamic_risk_single_element["concurrent_agent"]  # In order to store who is concurrent agent
                    #here dynamic_risk_list[index] = {"main_agent":"name", "concurrent_agent":"name", "name_main_agent":[],  "name_concurrent_agent":[], "dynamic_risk":[]}
                
                # Retrieve agent and concurrent agent skills 
                agent_skill = dynamic_risk_single_element["agent_skill"]
                concurrent_agent_skill = dynamic_risk_single_element["concurrent_skill"]
             
                #If pick_blue_box -> pick_blue_box_ + agent
                if "pick_blue_box" in dynamic_risk_single_element["agent_skill"]:
                    agent_skill += "_" + dynamic_risk_single_element["agent"]
                if "pick_blue_box" in dynamic_risk_single_element["concurrent_skill"]:
                    concurrent_agent_skill += "_" + dynamic_risk_single_element["concurrent_agent"]
                
                # Append dynamic_risk element 
                dynamic_risk_list[index][dynamic_risk_single_agent["_id"]].append(agent_skill) 
                dynamic_risk_list[index][dynamic_risk_single_element["concurrent_agent"]].append(concurrent_agent_skill) 
                dynamic_risk_list[index]["dynamic_risk"].append(dynamic_risk_single_element["dynamic_risk"])
                if "std_err" in dynamic_risk_single_element:
                    dynamic_risk_list[index]["std_err"].append(dynamic_risk_single_element["std_err"])
                else:
                    return SetBoolResponse(False,NOT_SUCCESSFUL) 
            print("-------------")
            print("Main agent: {}".format(dynamic_risk_single_agent["_id"]))
            print("Concurrent agent: {}".format(dynamic_risk_single_element["concurrent_agent"]))
            print(dynamic_risk_list[index])   
        ##### -------------------
        
        # fig, axs = plt.subplots(1, 2)
        # fig.suptitle('Dynamic Matrix')
        
        for index, single_agent_dynamic_risk in enumerate(dynamic_risk_list):
            # Retrieve agent name
            main_agent = single_agent_dynamic_risk["main_agent"]
            concurrent_agent = single_agent_dynamic_risk["concurrent_agent"]
            
            # Remove it for chart data
            single_agent_dynamic_risk.pop("main_agent",None)
            single_agent_dynamic_risk.pop("concurrent_agent",None)
            
            # Create a dataframe
            data = pd.DataFrame(single_agent_dynamic_risk)
            print(data)
            #sns.catplot(data=dati, x=main_agent, y="duration", hue=concurrent_agent)

            # ax = sns.catplot(data = data, kind="bar", x=main_agent, y="dynamic_risk", hue=concurrent_agent)
            # # plot time series with seaborn
            #ax = sns.barplot(data=data.dynamic_risk) #, err_style="unit_traces")

            # # Add std deviation bars to the previous plot
            mean = data.dynamic_risk
            std  = data.std_err
            # ax.errorbar(data.index, mean, yerr=std, fmt='-o')
            
            df = pd.DataFrame.from_dict({
                "mean": mean,
                "std": std
            }).reset_index()

            g = sns.FacetGrid(df, size=6)
            ax = g.map(plt.errorbar, "index", "mean", "std")
            ax.set(xlabel="", ylabel="")
            plt.show()
            # plt.figure(index)
            # ax = sns.barplot(data)
            # plt.title("Dynamic Risk Matrix for agent: {}".format(main_agent))
            # plt.savefig(self.results_folder_path + "dynamic_risk_agent_" + main_agent +".png")
    
    def singleDRCoefficientChart(self,request):   #Compute dynamic risk with concurrent and mean information
        """Method 

        Args:
            request (SetBoolRequest): 

        Returns:
            SetBoolResponse: 
        """
        import pandas as pd
        import seaborn as sns
        
        
        agents = self.getAgents()

        print(agents)
        
        n_samples = 1000
        for main_agent in agents:
            synergy_agent_values = pd.DataFrame(columns=['main_agent','main_task','concurrent_agent','concurrent_task','synergy'],dtype=object)
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
                return SetBoolResponse(False,NOT_SUCCESSFUL)
            
            for agent_synergy_element in results:
                concurrent_agent = agent_synergy_element["concurrent_agent"]
                main_task = agent_synergy_element["agent_skill"]
                concurrent_task = agent_synergy_element["concurrent_skill"]
                main_agent_label = main_agent
                paper = True
                if paper:
                    main_task = self.getPaperTaskName(self.checkPickPlaceWithAgent(agent_synergy_element["agent_skill"], main_agent))
                    concurrent_agent = self.getPaperAgentName(agent_synergy_element["concurrent_agent"])
                    concurrent_task = self.getPaperTaskName(self.checkPickPlaceWithAgent(agent_synergy_element["concurrent_skill"],agent_synergy_element["concurrent_agent"]))
                    main_agent_label = self.getPaperAgentName(main_agent)
                    # print(main_task)
                    # print(concurrent_task)
                    # print(main_task)
                    # print(concurrent_task)
                synergy_mean_value = agent_synergy_element["dynamic_risk"]
                synergy_std = agent_synergy_element["std_err"]
                dati = np.random.normal(synergy_mean_value,synergy_std,n_samples)
                
                extended_synergy = pd.DataFrame({'main_agent':main_agent,"main_task":main_task,"concurrent_agent":concurrent_agent,"concurrent_task":concurrent_task,"synergy":dati},dtype=object)
                
                synergy_agent_values = synergy_agent_values.append(extended_synergy,ignore_index=True)
            print("qui")
            sns.set_theme()
            sns.set(rc={'figure.figsize':(11,7)})
            # sns.catplot(data=dati, x=main_agent, y="duration", hue=concurrent_agent)

            ax = sns.catplot(data=synergy_agent_values, kind="bar", x="main_task", y="synergy", hue="concurrent_task",ci="sd")   
            ax.set_xlabels('Main Agent: ' + main_agent_label, fontsize=15,labelpad=15) # not set_label
            ax.fig.suptitle("Synergy Matrix Coefficients",y=1)
            ax.set_ylabels("Synergy coefficient value")
            ax._legend.set_title("Concurrent task ({})".format(concurrent_agent))
            # plt.xticks(rotation=, ha="right")
            self.results_folder_path="/home/samuele/projects/planning_ws/src/task-planner-interface/task_planner_statistics/file/"
            ax.figure.set_size_inches(11, 8)
            
            plt.savefig(self.results_folder_path + "dynamic_risk_coefficient_agent_" + main_agent +".png",)
            plt.savefig(self.results_folder_path+"test"+"dynamic_coefficient_risk_agent_"+".pdf",)
            
        # plt.figure()
        plt.show()
            # print(synergy_agent_values)      
              
        return SetBoolResponse(True,"ok")
        
        # task_results = dict()           
        # task_results["duration"] = []
        # for agent in agents:
        #     task_results[agent] =[]
        
    def make

    def checkPickPlaceWithAgent(self,task_name,agent):
        if task_name == "pick_blue_box":
            return task_name + "_" + agent
        else:
            return task_name

    def getPaperTaskName(self,task):
        if task == "pick_blue_box_human_right_arm":
            task_name = "Pick Blue Box (H)"
        elif task == "place_blue_box_human_right_arm":
            task_name = "Place Blue Box (H)"
        elif task == "pick_blue_box_ur5_on_guide":
            task_name = "Pick Blue Box (R)"
        elif task == "place_blue_box_ur5_on_guide":
            task_name = "Place Blue Box (R)"
        elif task == "place_blue_box_ur5_on_guide":
            task_name = "Place Blue Box"                       
        elif task == "pick_white_box":
            task_name = "Pick White Box"
        elif task == "place_white_box":
            task_name = "Place White Box"
        elif task == "pick_orange_box":
            task_name= "Pick Orange Box"
        elif task == "place_orange_box":
            task_name = "Place Orange Box"
        return task_name
    
    def getPaperAgentName(self,agent):
        if agent=="ur5_on_guide":
            agent_label = "Robot"
        elif agent=="human_right_arm":
            agent_label = "Human"
        return agent_label

def main():

    rospy.init_node("mongo_statistics")
    
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
    try:
        fig_folder=rospy.get_param("~fig_folder_path")   
    except KeyError:        
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("fig_folder_path") + END)
        fig_folder = "file"
        rospy.logerr("fig_folder set to: " + fig_folder)
        
    db_name = "agents_synergy"
    coll_properties_name = "tasks_properties_test"
    coll_results_name = "sinergy_results_test_long"  #"results_test"
    coll_duration_name = "durations_long_min"
    coll_risk_name = "dynamics_long_min"
    
    try:
        mongo_statistics = MongoStatistics(db_name,coll_properties_name,coll_results_name,coll_duration_name,coll_risk_name,fig_folder)
    except:
        return 0     #Connection to db failed 
    
       
    # Rosservice
    rospy.Service(COMPUTE_DURATION_SERVICE,SetBool,mongo_statistics.computeDurations)
    rospy.Service(COMPUTE_DYNAMIC_RISK_SERVICE,SetBool,mongo_statistics.computeDynamicRisk)
    COMPUTE_DYNAMIC_RISK_SERVICE_MIN = "mongo_statistics/compute_dynamic_risk_min"
    rospy.Service(COMPUTE_DYNAMIC_RISK_SERVICE_MIN,SetBool,mongo_statistics.computeDynamicRiskMin)

    rospy.Service(COMPUTE_DYNAMIC_RISK_SERVICE_V2,SetBool,mongo_statistics.computeDynamicRiskNoAddInfo) 
    rospy.Service(MAKE_CHART_SERVICE,SetBool,mongo_statistics.drawTimeline) 
    rospy.Service(MAKE_DR_CHART_SERVICE,SetBool,mongo_statistics.dynamicRiskChart) 
    rospy.Service("prova",SetBool,mongo_statistics.makeDurationChart) 
    rospy.Service("prova_regressione",SetBool,mongo_statistics.computeDynamicRiskWithUncertainty) 
    rospy.Service("prova_regressione_min",SetBool,mongo_statistics.computeDynamicRiskWithUncertaintyMin) 
    rospy.Service("prova_regressione_median",SetBool,mongo_statistics.computeDynamicRiskWithUncertaintyMedian) 


    rospy.Service("mongo_statistics/make_uncertainty_chart",SetBool,mongo_statistics.dynamicRiskUncertaintyChart) 
    
    rospy.Service("mongo_statistics/make_single_dr_coeff_chart",SetBool,mongo_statistics.singleDRCoefficientChart) 


    
    
    rospy.loginfo(READY)
    rospy.spin()

if __name__ == "__main__":
    main()