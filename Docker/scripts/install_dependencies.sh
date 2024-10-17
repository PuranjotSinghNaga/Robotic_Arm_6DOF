#!/bin/bash

echo "UPDATING SYSTEM...."
sudo apt-get update && apt-get upgrade -y

echo "INSTALLING REQUIRED DEPENDENCIES....."
sudo apt-get install -y \
    build-essential \
    python3-pip \
    git \
    vim \
    curl \
    wget \
    python3-vcstool \
    ros-humble-test-msgs \
    ros-humble-control-toolbox \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-moveit \
    ros-humble-visualization-msgs \
    python3-rosdep2 \
    ros-humble-ament-cmake


sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*
