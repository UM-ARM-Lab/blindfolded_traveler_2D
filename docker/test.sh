#/bin/bash

. /opt/ros/kinetic/setup.bash
. /root/catkin_ws/devel_isolated/setup.bash

cd ~/catkin_ws
catkin_make_isolated --pkg blindfolded_traveler_2d --catkin-make-args run_tests && catkin_test_results
