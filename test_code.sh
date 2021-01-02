#!/bin/bash

source ~/Desktop/bwi_ws/devel/setup.bash
# Launch simulator and grab PID
roslaunch bwi_launch multi_robot_simulation.launch &> simulator.log &
SIM_PID=$!
echo "Running simulator at PID = $SIM_PID"

echo "Wait 10 sec for simulator to be stabilized."
sleep 10

echo "Initiate task"
rostopic list
echo "task successfully end"

echo "Close simulator"
kill $SIM_PID
