#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
apt update
rosdep --rosdistro=humble update 


rm -rf /opt/ros/lcas
mkdir -p /opt/ros/lcas/src
chown -R lcas /opt/ros/lcas
cd /opt/ros/lcas/src
vcs import < /tmp/.devcontainer/lcas.repos
rosdep install --from-paths . -i -y
cd /opt/ros/lcas
colcon build


#cd /home/lcas/ws
#colcon build 
echo "source /opt/ros/lcas/install/setup.bash" >> ~/.bashrc

