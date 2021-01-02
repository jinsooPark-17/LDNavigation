#!/bin/bash

start=`date +%s`

source /bwi_ws/devel/setup.bash
# Launch simulator and grab PID
roslaunch bwi_launch multi_robot_simulation.launch &> simulator.log &
SIM_PID=$!
echo "Running simulator at PID = $SIM_PID"

cd /workspace

echo "Initiate task"
python record_random_waypoints.py $1
echo "task successfully end"

echo "Close simulator"
kill $SIM_PID

end=`date +%s`

echo "1 experiment took $((end-start))s!"