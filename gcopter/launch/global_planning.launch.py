from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('gcopter')
    return LaunchDescription([
        Node(package='tf2_ros', executable='static_transform_publisher', name='demo_fixed_frame',
             arguments=['--frame-id', 'world', '--child-frame-id', 'odom']),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('plot', default_value='true'),
        DeclareLaunchArgument('map', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'global_planning.yaml')),
        Node(package='mockamap', executable='mockamap_node', name='mockamap_node', output='screen',
             condition=IfCondition(LaunchConfiguration('map')),
             parameters=[{'seed': 1024, 'update_freq': 1.0, 'resolution': 0.25,
                          'x_length': 50, 'y_length': 50, 'z_length': 5, 'type': 1,
                          'complexity': 0.025, 'fill': 0.3, 'fractal': 1, 'attenuation': 0.1}],
             remappings=[('mock_map', '/voxel_map')]),
        Node(package='gcopter', executable='global_planning', name='global_planning_node', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'config', 'global_planning.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='rqt_plot', executable='rqt_plot', name='trajectory_plot',
             arguments=['/visualizer/speed/data', '/visualizer/total_thrust/data',
                        '/visualizer/tilt_angle/data', '/visualizer/body_rate/data'],
             condition=IfCondition(LaunchConfiguration('plot'))),
    ])
