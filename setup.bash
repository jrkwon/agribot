#!/bin/bash

####
## Assumption: you're at the 'agribot' directory.

##
# oscar main folder location
export AGRIBOT_PATH=$(pwd)

##
# set up catkin_ws with setup.bash
file_path=catkin_ws/devel/setup.bash
if [ -e "$file_path" ]; then
    source $file_path
fi

##
# add neural_net folder to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$(pwd)/neural_net
