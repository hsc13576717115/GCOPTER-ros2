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
        Node(package='mockamap', executable='mockamap_node', name='mockamap_node', output='screen', parameters=[{'seed': 511, 'update_freq': 1.0, 'resolution': 0.25, 'x_length': 50, 'y_length': 50, 'z_length': 5, 'type': 1, 'complexity': 0.035, 'fill': 0.3, 'fractal': 1, 'attenuation': 0.1}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(get_package_share_directory('mockamap'), 'config', 'rviz.rviz')], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
