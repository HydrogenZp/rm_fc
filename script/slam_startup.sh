#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/dynamicx/slam_ws/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311
# export ROS_IP=$(hostname -I | awk '{print $1}')
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1

sleep 5

roslaunch rm_fc odin_run.launch &
sleep 3

roslaunch rm_fc odom_to_acfly.launch &
sleep 1

roslaunch mavros acfly.launch respawn_mavros:=true &

wait
