import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Parâmetros da âncora medidos via RViz
    # x = -2.9394, y = 4.1843, z = 0.80
    
    x_arg = DeclareLaunchArgument('x', default_value='-2.9394')
    y_arg = DeclareLaunchArgument('y', default_value='4.1843')
    z_arg = DeclareLaunchArgument('z', default_value='0.80')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mycobot_anchor_publisher',
        arguments=[
            '--x', LaunchConfiguration('x'),
            '--y', LaunchConfiguration('y'),
            '--z', LaunchConfiguration('z'),
            '--yaw', LaunchConfiguration('yaw'),
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'mycobot_base_link'
        ]
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        static_tf_node
    ])
