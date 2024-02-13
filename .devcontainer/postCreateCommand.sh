#!/usr/bin/env bash

set -ex

echo "source /opt/ros/$ROS_DISTRO/setup.bash" >>~/.bashrc

colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

sudo rosdep init
just dep

just