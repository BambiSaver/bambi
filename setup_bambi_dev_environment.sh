#!/usr/bin/env bash

git submodule update --init --recursive

catkin init
wstool init src

sudo apt-get install python-catkin-tools -y

rosdep install --from install --from-paths src --ingor-src -y

src/mavros/install_geographiclib_datasets.sh

catkin build

echo "source devel/setup.bash" >> ~/.bashrc