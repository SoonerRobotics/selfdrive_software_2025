# Get local user
LOCAL_USER = $(whoami)

# Setup ROS2
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash

# Check if build is needed, if so, build
cd /home/$LOCAL_USER/selfdrive_software_2025/selfdrive_ws
UPSTREAM=${1:-'@{u}'}
LOCAL=$(git rev-parse @)
REMOTE=$(git rev-parse "$UPSTREAM")
BASE=$(git merge-base @ "$UPSTREAM")

if [ $LOCAL = $REMOTE ] || [ $REMOTE = $BASE ] || [ $LOCAL != $REMOTE ]; then
    echo "No Build Needed"
else
    echo "Build Needed"
    git pull
    colcon build
fi

# Launch
source /home/$LOCAL_USER/selfdrive_software_2025/selfdrive_ws/install/setup.bash
ros2 launch selfdrive_launch competition.xml