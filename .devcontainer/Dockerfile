ARG BASE_IMAGE=lcas.lincoln.ac.uk/ros:humble-staging
ARG ROS_DISTRO=humble

FROM ${BASE_IMAGE} AS devcontainer

RUN rm -rf /var/lib/apt/lists/* \
    && apt-get update --fix-missing \
    && apt-get -y install --no-install-recommends --fix-missing \
    ros-${ROS_DISTRO}-rviz2 \
    bash-completion \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

COPY ./ /workspace
WORKDIR /workspace
ENV ROS2_WS=/workspace

RUN sudo rm -rf log/ build/ install/

RUN rosdep update --rosdistro ${ROS_DISTRO} &&\
    sudo apt-get update && \
    rosdep install --from-paths ./src --ignore-src -r -y

# Do an initial build to speed up development
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --continue-on-error"

# ----------------------------------------------------------------------------------------------------
FROM devcontainer AS final

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

ENV MAP_FILE=/home/ros/workspace/src/topological_navigation/config/test_simple_tmap2.yaml 
CMD ["/bin/bash", "-lc", "source /workspace/install/setup.bash && exec ros2 launch topological_navigation topological_navigation.launch.py map_path:=$MAP_FILE"]
