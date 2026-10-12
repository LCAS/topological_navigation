#!/bin/bash

set -e

source /opt/ros/humble/setup.bash

# Only run apt/rosdep update if not done before (avoid hanging on rebuild)
MARKER="/home/lcas/.postCreateDone"
if [ ! -f "$MARKER" ]; then
    sudo apt-get update -qq || true
    rosdep --rosdistro=humble update --include-eol-distros || true
    cd /home/lcas/ws
    rosdep install --from-paths ./src -i -y --rosdistro=humble || true
    touch "$MARKER"
fi

cd /home/lcas/ws
colcon build --symlink-install

# Add workspace sourcing to bashrc (idempotent)
grep -q "install/setup.bash" ~/.bashrc 2>/dev/null || \
    echo "source /home/lcas/ws/install/setup.bash" >> ~/.bashrc

