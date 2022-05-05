#!/bin/bash
STRING="Executing training of Sharework-DBs"
echo $STRING
xterm -e "cd ~; roscd; roslaunch hrc_simulator_moveit_config demo.launch pipeline:=human_aware safety:=thor" &
sleep 10
xterm -e "cd ~; roscd; roslaunch hrc_mosaic_test pick_place_test.launch pipeline:=human_aware load_param:=1" &
sleep 90
xterm -e "cd ~; roscd; roslaunch task_planner_interface training_mongo_launcher.launch"


