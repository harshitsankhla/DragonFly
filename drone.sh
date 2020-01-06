#!/bin/bash

echo " _ _ _  ____        _      _                    _____                          _____      _               _ _ _ "
echo "| | | |/ __ \      | |    | |                  |  __ \                        / ____|    | |             | | | |"
echo "| | | | |  | |_   _| |_ __| | ___   ___  _ __  | |  | |_ __ ___  _ __   ___  | (___   ___| |_ _   _ _ __ | | | |"
echo "| | | | |  | | | | | __/ _\` |/ _ \ / _ \| '__| | |  | | '__/ _ \| '_ \ / _ \  \___ \ / _ \ __| | | | '_ \| | | |"
echo "|_|_|_| |__| | |_| | || (_| | (_) | (_) | |    | |__| | | | (_) | | | |  __/  ____) |  __/ |_| |_| | |_) |_|_|_|"
echo "(_|_|_)\____/ \__,_|\__\__,_|\___/ \___/|_|    |_____/|_|  \___/|_| |_|\___| |_____/ \___|\__|\__,_| .__/(_|_|_)"
echo "                                                                                                   | |          "
echo "                                                                                                   |_|          "
echo "This script will install - "
echo "1. ROS Melodic"
echo "2. librealsense"
echo "3. ceres-solver"
echo "4. ROS Packages for - bluefox | xsens-imu | realsense | VINS - Mono and Fusion | MAVROS | Octomap"
echo "5. Utilities - Sublime Text | Terminator | openssh | exFAT utils | tmux"
echo "6. MatrixVision Impact Driver for MonoCam"
echo -e '\n'
read -p "Press enter to continue...."

# paths and variables
BASE=$(pwd)

# shri ganesh
sudo apt-get update

# install librealsense debian packages
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
#sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo apt-get install librealsense2-udev-rules librealsense2-dkms librealsense2 librealsense2-utils librealsense2-dev librealsense2-dbg -y

# install some ubuntu basic utilities
sudo apt-get install terminator openssh-server exfat-fuse exfat-utils tmux -y

# install ROS Melodic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
sudo apt-get install ros-melodic-desktop-full -y

sudo rosdep init
rosdep update

echo "source /opt/ros/melodic/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc

sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y

# Sublime-Text
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update
sudo apt-get install sublime-text -y

# QGroundControl
mv ./UAV_tools/QGroundControl.AppImage $HOME
sudo chmod +x $HOME/QGroundControl.AppImage

# Ceres-Solver
mv $BASE/libraries/ceres-solver $HOME
sudo apt-get install libgoogle-glog-dev libatlas-base-dev libeigen3-dev -y
sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
sudo apt-get update
sudo apt-get install libsuitesparse-dev -y
cd $HOME
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver
make -j4
make test
sudo make install

# Some ROS Packages
sudo apt install ros-melodic-rgbd-launch -y
sudo apt-get install ros-melodic-octomap ros-melodic-octomap-mapping -y
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras -y

# Initialize ROS Workspace and add Packages
cd $HOME
mkdir -p dragonfly_ws/src
cd dragonfly_ws/src

mv $BASE/ROS_packages/* ./

cd ../
source /opt/ros/melodic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release -j8

echo "source ~/dragonfly_ws/devel/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc

# MAVROS Geographiclib Setup 
cd $HOME
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

# BASH Functions for easier calls
echo "bf(){
	LD_LIBRARY_PATH=/opt/mvIMPACT_Acquire/lib/x86_64:/opt/mvIMPACT_Acquire/Toolkits/expat/bin/x86_64/lib:$LD_LIBRARY_PATH roslaunch bluefox2 single_node.launch
}" >> $HOME/.bashrc
echo "imu(){
	sudo chmod 777 /dev/ttyUSB*
	roslaunch xsens_driver xsens_driver.launch
}" >> $HOME/.bashrc
echo "rs(){
	roslaunch realsense2_camera rs_rgbd.launch
}" >> $HOME/.bashrc
echo "px(){
	roslaunch mavros px4.launch
}" >> $HOME/.bashrc
echo "vmono(){
	roslaunch vins_estimator bf_xsens.launch
}" >> $HOME/.bashrc

# dumping bluefox2 camera YAML File
mkdir $HOME/.ros/camera_info
mv $BASE/extras/camera_yaml/* $HOME/.ros/camera_info/

# bluefox2 camera permissions settings
sudo cp $HOME/dragonfly_ws/src/bluefox2/mvIMPACT/script/51-mvbf.rules /etc/udev/rules.d/
sudo service udev reload

# bluefox2 USB driver
sudo chmod +x $BASE/extras/install_mvBlueFOX.sh
$BASE/extras/install_mvBlueFOX.sh
sudo rm /etc/ld.so.conf.d/acquire.conf
sudo ldconfig

# finishing touch
sudo usermod -a -G dialout $USER
sudo reboot
