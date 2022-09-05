from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():


    # 3 working modes:
  
    # 1 - the node works through a service (/get_obstacle_distance) which
    #     receives an OccupancyGrid as input and returns an ObstacleDistance
    #     msg.
  
    # 2 - The node uses the map_server service /map to get the static
    #     map only once, and send the created ObstacleDistance when
    #     receives a call to the service /get_static_obstacle_distance.
  
    # 3 - The node subcribes to the OccupancyGrid topic published by
    #     a local or a global costmap (the user must indicate the topic name)
    #     and publishes the created ObstacleDistance in the topic
    #     /obstacle_distance
    mode_arg = DeclareLaunchArgument(
      'mode', default_value="2"
    )

    thresh_arg = DeclareLaunchArgument(
      'obstacle_threshold', default_value="70"
    )

    costmap_arg = DeclareLaunchArgument(
      'costmap_topic', default_value="global_costmap/costmap"
    )

    return LaunchDescription([
        mode_arg,
        thresh_arg,
        costmap_arg,
        Node(
            package="obstacle_distance",
            executable="obstacle_distance_node",
            name="obstacle_distance_node",
            output="screen",
            parameters=[
                {'mode': LaunchConfiguration('mode')},
                {'obstacle_threshold': LaunchConfiguration('obstacle_threshold')},
                {'costmap_topic': LaunchConfiguration('costmap_topic')},
                {'use_sim_time': True}
            ]
            #remappings=[]
            #arguments=[]
        )
    ])