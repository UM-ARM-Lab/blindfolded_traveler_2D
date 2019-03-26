FROM ros:kinetic-ros-base

RUN mkdir -p ~/catkin_ws/src

RUN cd ~/catkin_ws/src && git clone https://github.com/UM-ARM-Lab/blindfolded_traveler_2D.git

RUN ~/catkin_ws/src/blindfolded_traveler_2D/docker/setup.sh
