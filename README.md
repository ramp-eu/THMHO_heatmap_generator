# betterfactory_sim

## Installation

Note: The multi-robot scenario currently works only on ROS-Melodic. See https://github.com/ros/robot_state_publisher/pull/139 for more details.

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

## Usage

with gazebo:
```
roslaunch factory_sim factory.launch sim:=true  
```
with real laser scanners:
```
roslaunch factory_sim factory.launch sim:=false
```
