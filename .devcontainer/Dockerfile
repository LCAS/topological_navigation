ARG BASE_IMAGE=ros:jazzy
ARG ROS_DISTRO=jazzy

FROM ${BASE_IMAGE} AS base

RUN rm -rf /var/lib/apt/lists/* \
    && apt-get update --fix-missing \
    && apt-get -y install --no-install-recommends --fix-missing \
    ros-${ROS_DISTRO}-rviz2 \
    bash-completion \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-${ROS_DISTRO}-rqt-graph \
    && rm -rf /var/lib/apt/lists/*

############################################################################################################
# Intermediate stage: extract only package.xml files so that the rosdep cache
# is only invalidated when package.xml files actually change, not on every
# source-tree modification. The bind-mount hash covers the whole tree, but the
# COPY --from below is content-addressed on /tmp/pkg-only alone.
FROM base AS pkg_xmls_only
RUN --mount=type=bind,target=/tmp/source-tree-ro,readonly \
    find /tmp/source-tree-ro -name "package.xml" | while IFS= read -r f; do \
        rel="${f#/tmp/source-tree-ro/}"; \
        mkdir -p "/tmp/pkg-only/$(dirname "$rel")"; \
        cp "$f" "/tmp/pkg-only/$rel"; \
    done




############################################################################################################
FROM base AS deps_installed
LABEL org.opencontainers.image.target="deps_installed"

COPY --from=pkg_xmls_only /tmp/pkg-only /tmp/pkg-only

# runs rosdep and uses JABAS_PLATFORM and JABAS_HARDWARE_INTERFACE to install only the correct dependencies for the platform, resolving dependencies recursively from the top-level package navstack_bringup. This is done in a separate stage so that the rosdep cache is only invalidated when package.xml files change, not on every source-tree modification.
RUN echo "Installing dependencies" && \
    cd /tmp/pkg-only && \
    echo "Running rosdep update..."  && \
    rosdep update --rosdistro ${ROS_DISTRO}  && \
    echo "Running apt-get update..."  && \
    apt-get update  && \
    echo "Running rosdep install..."  && \
    rosdep install --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y  || echo "rosdep install failed, continuing with flag to keep going"  && \
    rosdep install --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y -r

FROM deps_installed AS devcontainer
# do an initial build to speed up development
RUN --mount=type=bind,target=/workspace/src \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /workspace && \
    colcon build --continue-on-error

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc
RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /etc/bash.bashrc
RUN echo "alias build='(cd /workspace && colcon build)'" >> /etc/bash.bashrc


FROM devcontainer AS build
COPY ./ /workspace/src
WORKDIR /workspace
ENV ROS2_WS=/workspace

# Do an initial build to speed up development
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --continue-on-error"

# ----------------------------------------------------------------------------------------------------
FROM build AS final

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

ENV MAP_FILE=/workspace/src/topological_navigation/config/test_simple_tmap2.yaml 
CMD ["/bin/bash", "-lc", "source /workspace/install/setup.bash && exec ros2 launch topological_navigation topological_navigation.launch.py map_path:=$MAP_FILE"]