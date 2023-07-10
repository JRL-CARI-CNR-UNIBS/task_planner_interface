# Task Planner

## Aim

This package provides a task planner implementation based on a formalization as "Mixed Integer Linear Programming".

# To run:
- roslaunch hrc_simulator_configurations simulation.launch
- roslaunch hrc_mosaic_test skills_agents_synergy_test.launch
- roslaunch hrc_simulator_safety dynamic_ssm_on_area.launch 
- roslaunch hrc_simulator_safety dynamic_ssm.launch
- roslaunch hrc_mosaic_task_planning_interface task_planning.launch 
- roslaunch hrc_mosaic_task_planning_interface task_planner_interface_fake.launch hrc:=true is_human_real:=false
- rosrun task_planner reload_scene.py
- rosrun task_planner_statistics distance_monitoring.py