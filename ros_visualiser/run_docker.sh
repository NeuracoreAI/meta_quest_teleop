#!/bin/bash
# Simple script to run Meta Quest ROS2 visualization in Docker
# Usage: ./run_docker.sh [IP_ADDRESS] [FREQ] [PORT]

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Build if image doesn't exist
if [[ "$(docker images -q meta_quest_ros2 2> /dev/null)" == "" ]]; then
    echo "Building Docker image..."
    docker build -t meta_quest_ros2 ${SCRIPT_DIR}
fi

# Setup X11
xhost +local:docker 2>/dev/null

docker run -it --rm \
    --privileged \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e PYTHONPATH=/workspace \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ${SCRIPT_DIR}/..:/workspace:rw \
    -v /dev/bus/usb:/dev/bus/usb \
    meta_quest_ros2 \
    bash -c "cd /workspace && tmux new-session -d -s meta_quest \
    'source /opt/ros/rolling/setup.bash && \
    python3 ros_visualiser/ros2_tf_publisher.py; \
    bash' \; split-window -h 'source /opt/ros/rolling/setup.bash && \
    sleep 2 && rviz2 -d ros_visualiser/meta_quest_viewer.rviz; bash' \; attach"

