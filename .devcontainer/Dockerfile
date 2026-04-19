FROM ros:humble

RUN rm -rf /var/lib/apt/lists/* \
    && apt-get update --fix-missing \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends --fix-missing \
        ros-humble-desktop \
        bash-completion \
        python3-colcon-common-extensions \
        python3-pip \
        python3-vcstool \
        python3-rosdep \
        nano \
        git \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -rm -d /home/lcas -s /bin/bash -g root -G sudo -u 1001 lcas
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN echo "source /opt/ros/humble/setup.bash" >> /home/lcas/.bashrc

COPY *.sh /tmp/.devcontainer/
RUN bash /tmp/.devcontainer/install.sh

RUN mkdir -p /home/lcas/ws/src && ln -s /workspaces /home/lcas/ws/src/workspaces
WORKDIR /home/lcas/ws
