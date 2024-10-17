#!/bin/bash

# Create workspace
mkdir -p ~/abb_arm_ws/src

# Update rosdep
rosdep update

export CMAKE_PREFIX_PATH=/path/to/ament_cmake:$CMAKE_PREFIX_PATH

# Change to the workspace directory
cd ~/abb_arm_ws/src

echo "CLONING PROJECT"
git clone https://github.com/PuranjotSinghNaga/custom_robot_ros2_moveit2_template.git .


# Note: Do not source here, as it won't persist. 
# You can source it when you start your container or set it in your entry point if necessary.
