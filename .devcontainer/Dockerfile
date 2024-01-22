FROM lcas.lincoln.ac.uk/lcas/ros:humble

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y upgrade \
    && apt-get -y install --no-install-recommends ros-humble-desktop ros-humble-simulation bash-completion python3-colcon-common-extensions \
        nano syslog-ng \
        build-essential \
        curl \
        swig \
        python3-pip \
        ros-humble-rviz2 \
        python3-rosinstall-generator \
        wget

RUN useradd -rm -d /home/lcas -s /bin/bash -g root -G sudo -u 1001 lcas
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN echo "source /opt/ros/humble/setup.bash" >> /home/lcas/.bashrc

RUN pip3 install bloom

COPY *repos *.sh /tmp/.devcontainer/
RUN bash /tmp/.devcontainer/install.sh

RUN mkdir -p /home/lcas/ws/src && ln -s /workspaces /home/lcas/ws/src/workspaces
WORKDIR /home/lcas/ws
