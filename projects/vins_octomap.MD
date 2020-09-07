# VINS State Estimation and Outdoor Mapping using OctoMap

![ROS Drone Hardware Map](/extras/images/hardware.png)

## Manual Changes
- **Set your Monocular Camera Serial Number**
```
subl ~/dragonfly_ws/src/bluefox2/launch/single_node.launch
```
- **Set your Image and IMU topic for VINS-Mono** _(verify exact names by running - rostopic list)_
```
subl ~/dragonfly_ws/src/VINS-Mono/config/bf_xsens/bf_xsens_config.yaml
```

