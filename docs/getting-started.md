## Setting up the costmap generator node

The configuration file for costmap generator node is `betterfactory_sim/factory_sim/config/costmap_params.yaml`. An example of the configuration is shown below:
```
observation_sources: scan
scan: {sensor_frame: node_1, data_type: LaserScan, topic: /node_1/scan, marking: true, clearing: true,
       expected_update_rate: 0.4, max_obstacle_height: 1.0, min_obstacle_height: 0.0, obstacle_range: 10.0}
```
For each new laser scanner, an entry has to be added to `observation_sources` separated by spaces. For information about these parameters, please refer http://wiki.ros.org/costmap_2d/hydro/obstacles.

## Setting up the map server node

`map_server` requires the meta-data about the map, which is stored in a YAML file. The maps in .pgm format and its corresponding YAML file are located in `betterfactory_sim/factory_sim/maps` folder. The path to the YAML file is set in `factory_sim/launch/factory.launch`. For information about the `map_server` and the YAML file, please refer http://wiki.ros.org/map_server. 

## Setting up the heatmap generator node

`heatmap_generator` node aggregates the occupanpancy grid messages over a specified period and generates the heatmap for that time interval. This time interval can be configured in seconds with the argument `aggregation_time` in `factory_sim/launch/factory.launch`. The default value is 3600, i.e. 1 hour.