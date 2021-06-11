#!/bin/bash
#screen -dm bash -c "roscore; exec sh"
#screen -dm bash -c "roslaunch tiago_sim_robocup2021 tiago_sim_robocup2021.launch; exec sh"
screen -dm bash -c "source /eloquent_moveit_ws/install/setup.bash; export ROS_IP=172.19.0.5; export ROS_MASTER_URI=http://172.19.0.3:11311; ros2 launch ros1_bridge dedicated_bridges_launch.py"

#screen -dm bash -c "sleep 10; source /eloquent_moveit_ws/install/setup.bash; ros2 launch ros1_bridge dedicated_bridges_launch.py; exec sh"
sleep 3;
source /home/developer/robocup_melodic_ws/devel/setup.bash;
export ROS_IP=172.19.0.5;
export ROS_MASTER_URI=http://172.19.0.3:11311;
roslaunch gb_tiago_manipulation_demo clean_up_demo.launch