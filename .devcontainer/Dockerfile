FROM osrf/ros:jazzy-desktop-full


# Create workspace so that user own this directory
RUN mkdir -p /root/robot_ws/src
WORKDIR /root/robot_ws

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /root/.bashrc \
    && echo 'source /root/robot_ws/install/setup.bash' >> /root/.bashrc \
    && echo "export DISPLAY=unix:1" >> /root/.bashrc

# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
