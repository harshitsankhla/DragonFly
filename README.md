# DragonFly
Software Setup for Outdoor UAV at RRC-IIIT Hyderabad

![Hardware Map](/extras/hardware.png)

## Overview
For the development of an autonomous drone for outdoor environments. Online mapping, planning and control in unknown environments. GPS based goal selection. Octomap used for mapping. VINS used for state estimation. 

## Installation
_**NOTE:** Please run this installation only on a fresh formatted system with Ubuntu 16/18_

```
git clone https://github.com/harshitsankhla/DragonFly.git
cd DragonFly
sudo chmod +x drone.sh
. drone.sh
```
## Hardware Installed
  * mvBlueFox monocular camera
  * xsens IMU
  * Intel Realsense 415/435
  * Intel NUC Mini PC
  * Here+ RTK GPS
  * Pixhawk 4
  * TF Mini LiDAR

## Manual Changes
- **Set your Monocular Camera Serial Number**
```
subl ~/dragonfly_ws/src/bluefox2/launch/single_node.launch
```
- **Set your Image and IMU topic for VINS-Mono** _(verify exact names by running - rostopic list)_
```
subl ~/dragonfly_ws/src/VINS-Mono/config/bf_xsens/bf_xsens_config.yaml
```

## Tips and Tricks
- **Commands for Static Transform Publish between frames (will be used for attaching other sensor frames to world frame)**
```
rosrun tf static_transform_publisher x y z yaw pitch roll parent_frame child_frame period_in_ms
rosrun tf static_transform_publisher x y z qx qy qz qw parent_frame child_frame period_in_ms
```
- **If ROS gives permission denied error**
```
sudo rosdep fix-permissions
```
