FROM osrf/ros:jazzy-desktop-full

RUN mkdir -p /selfdrive/selfdrive_ws/src
RUN mkdir -p /selfdrive/setup
# RUN mkdir -p /selfdrive/deps

COPY selfdrive_ws/src /selfdrive/selfdrive_ws/src
COPY setup /selfdrive/setup
COPY vectorsecrets.txt /selfdrive/setup/vectorsecrets.txt
# COPY deps /selfdrive/deps

WORKDIR /selfdrive/setup
RUN /bin/bash -c "sudo chmod +x ./setup.sh"
RUN /bin/bash -c "./setup.sh"

WORKDIR /selfdrive/selfdrive_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build"
