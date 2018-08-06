#!/usr/bin/env bash

echo "##############################################"
echo " SCRIPT WILL FINISH WITH catkin build"
echo " script asks for root privileges for apt-get"
echo "##############################################"

echo


# checkout all submodules
git submodule update --init --recursive


# init catkin workspace
catkin init
wstool init src

# install stuff
sudo apt-get install python-catkin-tools -y
rosdep install --from install --from-paths src --ingor-src -y
src/mavros/install_geographiclib_datasets.sh


# ADD TO BASHRC
path="$(pwd)/devel/setup.bash"

# assume to under home directory and replacing /home/$USER/ with ~/
path=${path#*/}
path=${path#*/}
path="${path#*/}"

if [ $(cat ~/.bashrc | grep -c $path) -lt 1 ] ; then
  echo "source ~/$path" >> ~/.bashrc;
fi


#build
catkin config -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_ARG1=-std=c++11 -D__cplusplus=201103L -D__GXX_EXPERIMENTAL_CXX0X__=1
catkin build

# even
source devel/setup.bash