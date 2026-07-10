#!/usr/bin/env python3
"""Launch RViz with the Lite3 AEDE display layout."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = LaunchConfiguration('rviz_config')

    default_config = PathJoinSubstitution([
        FindPackageShare('lite3_bridge'),
        'config',
        'lite3_aede.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=default_config),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
