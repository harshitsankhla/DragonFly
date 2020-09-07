# DragonFly
Outdoor UAV Projects at RRC-IIIT Hyderabad

## Overview
At the Robotics Research Center (IIIT Hyderabad) we are drivng research in the development of autonomous drones for outdoor environments. This repository contains all the relevant information to recreate the setups and projects we are developing. We are deeply thankful to the open-source community and the tons of developers involved in developing software and hardware that have made this project possible, hence we feel obliged to follow the same footsteps and hope our work encourages people.

## Installation
This script will setup your companion computer with all the necessary software and packages used for recreating our projects
_**NOTE:** Please run this installation only on a fresh formatted system with Ubuntu 18 or later_

```
git clone https://github.com/harshitsankhla/DragonFly.git
cd DragonFly
```
For ROS Based Platform
```
sudo chmod +x drone_ros.sh
. drone_ros.sh
```
For Non-ROS Based Platform
```
sudo chmod +x drone_noros.sh
. drone_noros.sh
```

## Exploring the Repository
1. drone_designs - this contains BOM and assembly instructions for the different drones we have in our lab
2. projects - this contains instructions for different applications we have developed using drones
3. scripts - examples, test codes, debugging code for ArduPilot/PX4 based vehicles
4. ROS_packages - installed on the companion computer and used across the ROS based projects
5. libraries - 3rd party libraries required for projects
6. extras - some general configuration files and installation scripts, mainly used during setup 
