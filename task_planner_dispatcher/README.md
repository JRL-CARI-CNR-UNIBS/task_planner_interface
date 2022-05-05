This package provides nodes to read task plans from yaml files and dispatch the tasks.
Tasks are dispatched by publishing messages of type MotionTaskExecutionRequestArray and feedbacks are received by subscribing a topic of type MotionTaskExecutionFeedback.

Available nodes:

- dispatcher_single : dispatches task for a single agent (e.g. robot)
- dispatcher_double_sync : dispatches tasks for human and robot; waits for both feedbacks before sending new tasks
- dispatcher_double_async : dispatches tasks for human and robot;independently from each other's feedback

## Launchers
```
roslaunch task_planner_dispatcher task_dispatcher.launch
```
Launch dispatcher_single node with parameters.

```
roslaunch task_planner_dispatcher task_dispatcher_duo_async.launch
```
Launch dispatcher_double_async node with parameters.

