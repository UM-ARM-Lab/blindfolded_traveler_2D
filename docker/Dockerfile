FROM ros:kinetic-ros-base

RUN mkdir -p ~/catkin_ws/src


RUN apt update -yq\
    && apt -yqq install \
    libeigen3-dev \
    libz-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN cd /usr/src/gtest && cmake CMakeLists.txt && make && cp *.a /usr/lib