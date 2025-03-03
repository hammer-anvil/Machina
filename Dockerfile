# Use Ubuntu 24.04 as the base image
FROM ubuntu:24.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    ROS_DISTRO=jazzy \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    COLCON_LOG_LEVEL=INFO

# Update package list and install base dependencies
RUN apt update && apt install -y \
    software-properties-common curl wget lsb-release gnupg2 git build-essential \
    python3 python3-pip python3-flake8 python3-pytest python3-empy python3-yaml \
    libgl1 mesa-utils x11-apps && \
    rm -rf /var/lib/apt/lists/*

# Add ROS 2 sources manually (since Ubuntu 24.04 does not include ROS 2 packages by default)
RUN echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list && \
    curl -sSL 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key' | apt-key add - && \
    apt update

# Install ROS 2 core and development tools
RUN apt install -y \
    ros-${ROS_DISTRO}-desktop \
    python3-rosdep python3-colcon-common-extensions python3-rosinstall-generator python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# Initialize and update rosdep
RUN rosdep init && rosdep update

# Setup ROS 2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]

# Set working directory inside the container
WORKDIR /ros2_ws

# Copy the machina_metrology_task package
COPY ./machina_metrology_task /ros2_ws/src/machina_metrology_task/

# Ensure the entrypoint script is executable
RUN chmod +x /ros2_ws/src/machina_metrology_task/docker_entrypoint.bash

# Source ROS environment and build the workspace using colcon
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /ros2_ws && colcon build --symlink-install"

# Set default entrypoint
ENTRYPOINT ["/ros2_ws/src/machina_metrology_task/docker_entrypoint.bash"]
CMD ["bash"]
