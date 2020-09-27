# VINS State Estimation and Outdoor Mapping using OctoMap (ROS)
This project is on outdoor mapping. Our aim is to make an [Octomap](https://octomap.github.io/) which is a 3D occupancy map using Visual-Inertial Odometry and, optionally, global fusion with GPS. The map created can be later used for navigating drones in the environment, by way-point extraction through path planning algorithms. 
![](/extras/gifs/vins_octomap.gif)

## Intro
To build an Octomap, we need 2 important pieces of information. A 3D perception of the environment and our pose/trajectory to register a map in reference to a fixed frame. For this project, we obtain 3D PointCloud from a Realsense D415 and VI-Odometry through a BlueFox-Xsens (Monocular Camera-IMU) combination, however, the pipeline will remain the same for other hardware configurations you may wish to explore. The camera is hardware-triggered via the IMU. Time-synchronisation is extremely important in VI-Odometry, and ideally should be within an order of 1ms, else you may observe significant drift in trajectory. Time-sync is reaquired for GPS as well if you choose to do global fusion with the VI-Odometry.

## Hardware Setup
Following is the setup designed for this project, mounted on [Lucifer](https://github.com/harshitsankhla/DragonFly/blob/master/drone_designs/lucifer.md).
<p align="center">
  <img src="/extras/images/vins_setup.jpg">
</p>
The BlueFox camera is hardware triggered from the Xsens IMU. This connection has to be created separately depending upon the cables you have. All components are connected via USB to the NUC.

## Workspace Configuration
***NOTE:*** You need to have your DragonFly ROS Platform setup as mentioned [here](https://github.com/harshitsankhla/DragonFly#installation).

- **Set your BlueFox2 Monocular Camera Serial Number -**
https://github.com/harshitsankhla/DragonFly/blob/c85165c5190ae670d9ef7308a3ab1935f050d34e/ROS_packages/bluefox2/launch/single_node.launch#L3
*The serial number is usually provided by the manufacturer, or marked on its circuit-board, else, you can connect it to your PC and run the [bluefox2 single_node launch file](https://github.com/harshitsankhla/DragonFly/blob/master/ROS_packages/bluefox2/launch/single_node.launch), which will throw a device error with the serial number of the device actually connected.*

- **Set your Image and IMU topics for VINS-Fusion -**
https://github.com/harshitsankhla/DragonFly/blob/master/ROS_packages/VINS-Fusion/config/bf_xsens/bf_xsens_config.yaml
*Since we are using VINS-Fusion in monocular mode, you can either leave the [image1](https://github.com/harshitsankhla/DragonFly/blob/c85165c5190ae670d9ef7308a3ab1935f050d34e/ROS_packages/VINS-Fusion/config/bf_xsens/bf_xsens_config.yaml#L10) topic empty or set it to be the same as the image0 topic.*

- **Add your camera calibration info for VINS-Fusion -**
https://github.com/harshitsankhla/DragonFly/blob/master/ROS_packages/VINS-Fusion/config/bf_xsens/bf_camera.yaml
*Change the camera parameters according to your own calibration results.*

- **Octomap Configuration  -**
*You need to configure the frame in which the map and odometry is to be published and define the transform between the PointCloud and Odometry Source so the PointCloud registers correctly. There are a couple other parameters you can change, from default, to improve performance. You can directly use the launch file provided here which is a decently tested.*

## Map Building Step 1 - Data Collection


## Map Building Step 2 - Visualization
