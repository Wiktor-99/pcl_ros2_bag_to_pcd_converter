ARG ROS_DISTRO=humble
FROM osrf/ros:$ROS_DISTRO-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-rosbag2 \
    ros-$ROS_DISTRO-rosbag2-cpp \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
COPY ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/bin/bash", "/ros_entrypoint.sh"]
CMD ["bash"]

WORKDIR /app
COPY pcl_bag_to_pcd/ src/pcl_bag_to_pcd

RUN cd /app && source /opt/ros/$ROS_DISTRO/setup.bash && colcon build
