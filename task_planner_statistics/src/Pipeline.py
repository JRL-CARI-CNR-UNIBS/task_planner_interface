#! /usr/bin/env python3

class Pipeline:
    
    @staticmethod
    def durationComputationPipeline(coll_durations_name : str) -> list:
        
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
        '$out': coll_durations_name
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
        '$out': coll_durations_name
        }
        ]
        
        return pipeline
    
    
    @staticmethod
    def createUtilsResultsPipeline(coll_durations_name,utils_results_name):
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
                'from': coll_durations_name, 
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
            '$out': utils_results_name
        }
        ]
                
        return pipeline
    
    @staticmethod
    def taskListPipeline():
        pipeline =[
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
        return pipeline
    
    @staticmethod
    def dynamicRiskPipeline(utils_results_name):
        pipeline = [
        {
            '$lookup': {
                'from': utils_results_name, 
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
                'from': utils_results_name, 
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
                'from': utils_results_name, 
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
                'from': utils_results_name, 
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
        return pipeline
    
    @staticmethod
    def groupedDynamicRiskPipeline():
        pipeline = [        # Pipeline to group/separate dynamic risk elements by agent  
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
        return pipeline
    
    @staticmethod
    def dynamicRiskNoAddInfo(coll_durations_name,
                             coll_results_name):
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
                'from': coll_durations_name, 
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
                'from': coll_results_name, 
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
                'from': coll_results_name, 
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
                'from': coll_results_name, 
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

        
        return pipeline
    
    @staticmethod
    def dynamicRiskForChart(utils_results_name):
        pipeline = Pipeline.dynamicRiskPipeline(utils_results_name)
        
        additional_part = [
            {
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
        }]
        pipeline.extend(additional_part)   
        return pipeline