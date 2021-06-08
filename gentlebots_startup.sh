#!/bin/bash
screen -dm bash -c "roscore; exec sh"
#screen -dm bash -c "roslaunch tiago_sim_robocup2021 tiago_sim_robocup2021.launch; exec sh"
#screen -dm bash -c "sleep 10; source /eloquent_moveit_ws/install/setup.bash; ros2 launch ros1_bridge dedicated_bridges_launch.py; exec sh"
sleep 3;
source /eloquent_moveit_ws/install/setup.bash;
ros2 launch ros1_bridge dedicated_bridges_launch.py