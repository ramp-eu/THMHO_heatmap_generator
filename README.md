# betterfactory_sim

## Installation

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
