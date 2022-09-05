# Obstacle Distance Manager

A metapackage that contains the methods to compute and to publish an occupancy grid based on a distance transform function to obstacles. 


## Dependencies

 - OpenCV


## Functioning

3 working modes:

1. the node works through a service (*/get_obstacle_distance*) which receives an OccupancyGrid as input and returns an ObstacleDistance msg.

2. The node uses the map_server service */map* to get the static map only once, and send the created ObstacleDistance when receives a call to the service */get_static_obstacle_distance*.

3. The node subcribes to the OccupancyGrid topic published by a local or a global costmap (the user must indicate the topic name) and publishes the created ObstacleDistance in the topic */obstacle_distance*


## Parameters

- `mode`: working mode.
- `obstacle_threshold`: values of the input occupancy grid above this threshold are considered as obstacles (Default: 70).
- `costmap_topic`: name of the costamp topic to get the occupancy grid from (only used in mode *3*). (Default: */global_costmap/costmap*).