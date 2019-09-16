#!/usr/bin/bash

# paths and variables
BASE=$(pwd)

# customary commands
sudo apt-get update

# install librealsense debian packages
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u

sudo apt-get install librealsense2-udev-rules librealsense2-dkms librealsense2 librealsense2-utils librealsense2-dev librealsense2-dbg

# install ubuntu basic utilities
sudo apt-get install terminator git openssh-server exfat-fuse exfat-utils

# install ROS Kinetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full

sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source $HOME/.bashrc

sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install ros-kinetic-rgbd-launch

# Sublime-Text
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update
sudo apt-get install sublime-text

# QGroundControl
mv ./QGroundControl.AppImage $HOME
sudo chmod +x $HOME/QGroundControl.AppImage

# Ceres-Solver
mv $BASE/ceres-solver $HOME
sudo apt-get install libgoogle-glog-dev libatlas-base-dev libeigen3-dev
sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
sudo apt-get update
sudo apt-get install libsuitesparse-dev
cd $HOME
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver
make -j4
make test
sudo make install

# Initialize ROS Workspace and add Packages
cd $HOME
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace

mv $BASE/bluefox2 ./
mv $BASE/camera_base ./
mv $BASE/ddynamic_reconfigure ./
mv $BASE/ethzasl_xsens_driver ./
mv $BASE/realsense-ros ./
mv $BASE/VINS-Fusion ./
mv $BASE/VINS-Mono ./
mv $BASE/octomap_mapping ./

cd ../
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install

echo "source ~/catkin_ws/devel/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc

# OctoMap


# MAVROS
cd $HOME
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo chmod +x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh

# finishing touch
sudo reboot

