#!/bin/bash

#User variables:
export ROBOT=youbot
export SIM=true


##### DO NOT EDIT BELOW THIS LINE #####

# Run the ros setup script
source devel/setup.bash

export WORKSPACE=`pwd`
export SRC=$WORKSPACE"/src"

echo "Robot platform       :" $ROBOT
echo "Workspace folder     :" $WORKSPACE
echo "Workspace src folder :" $SRC
echo "Simulation           :" $SIM
