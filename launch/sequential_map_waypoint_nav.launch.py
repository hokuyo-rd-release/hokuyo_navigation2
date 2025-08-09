from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name', # 自身のパッケージ名に置き換えてください
            executable='sequential_map_waypoint_nav',
            name='sequential_map_waypoint_nav',
            parameters=[
                {'map_waypoint_list_csv': 'map_waypoints.csv'},
                {'map_base_path': os.path.join(get_package_share_directory('hokuyo_navigation2'), 'maps')},
                {'waypoint_base_path': os.path.join(get_package_share_directory('waypoint_manager'), 'waypoints')},
                {'waypoint_tolerance_position': 0.2},
                {'waypoint_tolerance_orientation': 0.2},
            ],
            output='screen'
        )
    ])