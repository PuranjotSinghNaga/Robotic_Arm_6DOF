# Use the official ROS Humble image as the base
FROM ros:humble

# Install sudo and vim, along with other utilities
RUN apt-get update && apt-get install -y \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user (e.g., rosuser) and add it to the sudo group
RUN useradd -m -s /bin/bash rosuser && \
    echo "rosuser ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Switch to the new user
USER rosuser
# Set up the environment for ROS
ENV DISPLAY=host.docker.internal:0.0
ENV ROS_DISTRO=humble

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
# Installing necessary packages 
RUN sudo apt-get update && sudo apt-get install -y \
    vim \
    python3-vcstool \
    ros-humble-test-msgs \
    ros-humble-control-toolbox \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-moveit \
    ros-humble-visualization-msgs \
    python3-rosdep2 \
    wget \
    python3-colcon-common-extensions \
    ros-humble-ament-cmake \
    ros-humble-ament-lint \
    ros-humble-ament-cmake-auto \
    ros-humble-ament-cmake-core \
    ros-humble-ament-cmake-test

RUN rosdep update 
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

RUN mkdir -p /home/rosuser/abb_arm_ws/src

WORKDIR /home/rosuser/abb_arm_ws/src
RUN git clone https://github.com/PuranjotSinghNaga/custom_robot_ros2_moveit2_template.git /home/rosuser/abb_arm_ws/src/
WORKDIR /home/rosuser/abb_arm_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/ros/humble && \
    colcon build"
RUN echo "source install/setup.bash" >> ~/.bashrc



# Run bash by default
CMD ["bash"]
