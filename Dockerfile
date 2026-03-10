FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rosidl-default-generators \
    ros-humble-nav2-msgs \
    ros-humble-nav2-route \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-description \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace/src/topological_navigation

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /workspace && \
    colcon build --symlink-install

CMD ["bash", "-lc", "source /opt/ros/$ROS_DISTRO/setup.bash && source /workspace/install/setup.bash && ros2 launch topological_navigation turtlebot_route_compose.launch.py"]
