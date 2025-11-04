# Dockerfile for Meta Quest Reader with ROS2 Rolling
FROM ros:rolling

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=rolling

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    android-tools-adb \
    android-tools-fastboot \
    usbutils \
    vim \
    wget \
    curl \
    git \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-default-plugins \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir --break-system-packages \
    numpy \
    scipy \
    pure-python-adb

# Create workspace
WORKDIR /workspace

# Note: Actual code will be volume-mounted at runtime
# This keeps the image small and allows live code editing

# Source ROS2 in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "export PYTHONPATH=/workspace:\$PYTHONPATH" >> ~/.bashrc

# Set up ADB server
RUN mkdir -p ~/.android && echo "# ADB config" > ~/.android/adb_usb.ini

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS2\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
\n\
# Add workspace to Python path\n\
export PYTHONPATH=/workspace:$PYTHONPATH\n\
\n\
# Start ADB server\n\
adb start-server 2>/dev/null || true\n\
\n\
# Execute command\n\
exec "$@"\n\
' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

