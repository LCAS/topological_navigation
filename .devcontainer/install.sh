#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
apt-get update
rosdep init || true
rosdep --rosdistro=humble update

rm -rf /var/lib/apt/lists/*

