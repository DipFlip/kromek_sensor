FROM ros:humble-ros-base

LABEL maintainer="SLAM Dunk"
LABEL description="ROS2 driver for Kromek Sigma50 using HIDAPI (no kernel module required)"

# Install HIDAPI and build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    libhidapi-dev \
    libhidapi-libusb0 \
    libudev-dev \
    libusb-1.0-0-dev \
    pkg-config \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    ros-${ROS_DISTRO}-launch \
    ros-${ROS_DISTRO}-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# Copy ROS2 package from current directory
COPY . /ros2_ws/src/kromek_ros2_hidapi

# Build the package
WORKDIR /ros2_ws
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
    --packages-select kromek_ros2_hidapi \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release" && \
    rm -rf build logs

# Setup bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
echo "========================================="\n\
echo "Kromek Sigma50 HIDAPI ROS2 Driver"\n\
echo "========================================="\n\
echo "ROS Distribution: ${ROS_DISTRO}"\n\
echo "ROS Domain ID: ${ROS_DOMAIN_ID}"\n\
echo "RMW Implementation: ${RMW_IMPLEMENTATION}"\n\
echo "========================================="\n\
echo ""\n\
exec "$@"\n' > /entrypoint.sh && chmod +x /entrypoint.sh

# Set environment variables
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
