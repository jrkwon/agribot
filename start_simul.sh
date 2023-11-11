#!/bin/bash

cd catkin_ws
catkin_make
cd ..

source setup.bash

if [ -z "$1" ]; then
    echo "Starting Gazebo with default world..."  
    roslaunch scout_gazebo scout_playpen.launch
else
    echo "Starting Gazebo with $1..."  
    roslaunch scout_gazebo $1.launch