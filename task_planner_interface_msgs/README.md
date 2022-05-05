## Messages

 - MotionTaskExecutionRequestArray.msg : array of MotionTaskExecutionRequest. Subscribed by robot_node and human_node in task_planner_interface. The first element is sent to the robot/human for execution
 - MotionTaskExecutionRequest.msg : defines the properties of a task to be executed.
 - MotionTaskExecutionFeedback.msg : published by robot_node and human_node. It contains SUCCESS/FAILURE of the requested task.