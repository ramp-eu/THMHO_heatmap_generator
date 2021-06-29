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
catkin build
source ~/betterfactory_ws
```

## Usage

```
roslaunch factory_sim factory.launch --screen map_file:=~/betterfactory_ws/src/betterfactory_sim/factory_sim/maps/map.yaml
```
