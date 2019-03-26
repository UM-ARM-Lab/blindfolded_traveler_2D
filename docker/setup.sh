#/bin/bash

. /opt/ros/kinetic/setup.bash

cd ~/catkin_ws/src/
git clone https://github.com/UM-ARM-Lab/arc_utilities.git
cd ~/catkin_ws/src/arc_utilities
git checkout CleanUpDijkstras

cd ~/catkin_ws/src/
git clone https://github.com/UM-ARM-Lab/unknown_graph_planner.git
cd ~/catkin_ws/src/unknown_graph_planner
git checkout BlindfoldedTraveler

cd ~/catkin_ws
catkin_make
