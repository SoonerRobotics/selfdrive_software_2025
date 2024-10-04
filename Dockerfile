FROM osrf/ros:jazzy-desktop-full

RUN mkdir -p /selfdrive/selfdrive_ws/src

COPY selfdrive_ws/src /selfdrive/selfdrive_ws/src

WORKDIR /selfdrive/selfdrive_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build"