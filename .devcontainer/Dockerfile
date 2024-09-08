FROM osrf/ros:jazzy-desktop-full

ARG USER=ros

# Create a new user and set it as sudoer
RUN useradd -m -s /bin/bash $USER && \
    echo "$USER ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    chown -R $USER /home/$USER && \
    usermod -aG sudo $USER

# Switch to the new user
USER $USER

# Create workspace so that user own this directory
RUN mkdir -p ~/robot_ws/src
WORKDIR '/home/'$USER'/robot_ws'

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> ~/.bashrc \
    && echo 'source ~/robot_ws/install/setup.bash' >> ~/.bashrc \
    && echo "export DISPLAY=unix:1" >> ~/.bashrc

# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
