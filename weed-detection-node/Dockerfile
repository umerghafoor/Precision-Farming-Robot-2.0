FROM ros:jazzy

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --break-system-packages -r /tmp/requirements.txt

# Create workspace
WORKDIR /root/ros2_ws

# Copy package files
COPY . /root/ros2_ws/src/weed_detection_node

# Build the ROS2 package
RUN . /opt/ros/jazzy/setup.sh && \
    cd /root/ros2_ws && \
    colcon build --symlink-install

# Source the workspace
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Set entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["/bin/bash"]

