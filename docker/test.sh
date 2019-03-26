#/bin/bash

. /opt/ros/kinetic/setup.bash
. /root/catkin_ws/devel/setup.bash

cd ~/catkin_ws
catkin_make_isolated run tests && catkin_test_results
