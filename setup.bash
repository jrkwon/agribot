#!/bin/bash

####
## Assumption: you're at the 'agribot' directory.

##
# oscar main folder location
export AGRIBOT_PATH=$(pwd)

##
# set up catkin_ws with setup.bash
source ./catkin_ws/devel/setup.bash

##
# add neural_net folder to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages:$(pwd)/neural_net
