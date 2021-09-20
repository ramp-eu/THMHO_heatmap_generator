# Installation

## Requirements
The installation instructions are meant for devices with
- Ubuntu 20.04 or higher
- [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

## Installation instructions
```
source /opt/ros/<ROS-DISTRO>/setup.bash
mkdir -p ~/betterfactory_ws/src
cd ~/betterfactory_ws/src
git clone git@github.com:ipa320/betterfactory_sim.git
cd ~/betterfactory_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin config --install
catkin build
source ~/betterfactory_ws/install/setup.bash
```
