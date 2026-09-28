from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(package='tf2_ros', executable='static_transform_publisher', name='demo_fixed_frame',
             arguments=['--frame-id', 'world', '--child-frame-id', 'odom']),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='mockamap', executable='mockamap_node', name='mockamap_node', output='screen', parameters=[{'seed': 510, 'update_freq': 1.0, 'resolution': 0.1, 'x_length': 20, 'y_length': 20, 'z_length': 2, 'type': 3, 'road_width': 0.5, 'add_wall_x': 0, 'add_wall_y': 0, 'maze_type': 1}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(get_package_share_directory('mockamap'), 'config', 'rviz.rviz')], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
