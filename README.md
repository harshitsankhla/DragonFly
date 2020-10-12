# DragonFly
Outdoor UAV Projects at RRC-IIIT Hyderabad. Sponsored by [MeitY](https://www.meity.gov.in/)

## Overview
At the Robotics Research Center (IIIT Hyderabad) we are drivng research in the development of autonomous drones for outdoor environments. This repository contains all the relevant information to recreate the setups and projects we are developing. We are deeply thankful to the open-source community and the tons of developers involved in developing software and hardware that have made this project possible, hence we feel obliged to follow the same footsteps and hope our work encourages people.

## Installation
We have 2 companion computer setups for our projects. Depending upon your hardware budget and application, you can setup either one. ROS applications are generally more computationally expensive. Choose wisely.

_**For ROS Platform** -_
This script will setup your companion computer with all the necessary software and packages used for recreating our ROS-based projects.
_NOTE: Please run this installation only on a fresh formatted system with Ubuntu 18 or later_
```
git clone https://github.com/harshitsankhla/DragonFly.git
cd DragonFly
sudo chmod +x scripts/drone_ros.sh
. scripts/drone_ros.sh
```
_**For Non-ROS Platform** -_
Follow [this page](https://ardupilot.org/dev/docs/apsync-intro.html) to setup APSync and connect your companion computer to the flight controller.

## Exploring the Repository
1. drone_designs - this contains BOM and assembly instructions for the different drones we have in our lab
2. projects - this contains instructions for different applications we have developed using drones
3. scripts - setup, test, debugging scripts for vehicles
4. ROS_packages - installed on the companion computer and used across the ROS based projects
5. libraries - 3rd party libraries required for projects
6. extras - some general configuration files and installation scripts, mainly used during setup 

## Datasets and Videography
You can see our flights, results and data for your own testing here - [DragonFly OneDrive](https://iiitaphyd-my.sharepoint.com/:f:/g/personal/harshit_sankhla_research_iiit_ac_in/EvRMo_K0h-xIg3kN_zwN-CYBwskYUOsObzIfEdtaNGhj4w?e=ZrnOyw)
