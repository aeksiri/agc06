#! /bin/bash

source /opt/ros/kinetic/setup.bash
source /home/nvidia/denso_agc03_ws/devel/setup.bash

export ROS_MASTER_URI=http://agc03tx2:11311
export ROS_HOSTNAME=agc03tx2
export LINOLIDAR=sick_tim310
export LINOBASE=2wd

roslaunch linorobot agc03_bringup.launch
