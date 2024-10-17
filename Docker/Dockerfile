FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV USERNAME=myuser
ENV USER_HOME=/home/$USERNAME

# Create a new user and add to the sudo group
RUN useradd -m -s /bin/bash $USERNAME && \
    echo "$USERNAME:password" | chpasswd && \
    adduser $USERNAME sudo && \
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Copy the install dependencies script into the container
COPY ./scripts/install_dependencies.sh /tmp/install_dependencies.sh
COPY ./scripts/setup_ros.sh /tmp/setup_ros.sh

# Make the script executable
RUN chmod +x /tmp/install_dependencies.sh
RUN chmod +x /tmp/setup_ros.sh

# Switch to the new user
USER $USERNAME
WORKDIR $USER_HOME

# Run the dependencies installation script
RUN /tmp/install_dependencies.sh
RUN /tmp/setup_ros.sh
WORKDIR /home/myuser/abb_arm_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source ROS environment in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Set the entry point to bash with a command to keep the terminal open
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec bash"]
