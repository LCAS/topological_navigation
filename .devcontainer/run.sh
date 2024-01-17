#!/bin/bash

set -e

WORKSPACE="`pwd`"

source /opt/ros/lcas/install/setup.bash
sudo apt update
rosdep --rosdistro=humble update 


# sudo rm -rf /opt/ros/lcas
# sudo mkdir -p /opt/ros/lcas/src
# sudo chown -R lcas /opt/ros/lcas
# cd /opt/ros/lcas/src
# vcs import < $WORKSPACE/.devcontainer/lcas.repos
# rosdep install --from-paths . -i -y
# cd /opt/ros/lcas
# colcon build


cd /home/lcas/ws
rosdep install --from-paths ./src -i -y
colcon build --symlink-install
echo "source /home/lcas/ws/install/setup.bash" >> ~/.bashrc

