ARG ROS_DISTRO=humble
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-rosbag2 \
    ros-$ROS_DISTRO-rosbag2-cpp \
    libpcl-dev

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
COPY ros_entrypoint.sh /
ENTRYPOINT ["../ros_entrypoint.sh"]
RUN chmod +x /ros_entrypoint.sh

WORKDIR /app
COPY pcl_bag_to_pcd/ src/pcl_bag_to_pcd

RUN cd /app && source /opt/ros/$ROS_DISTRO/setup.bash && colcon build