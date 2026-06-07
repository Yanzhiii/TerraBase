#!/usr/bin/env python3
"""Real Go2 + AEDE bringup using the original go2_robot_sdk."""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _share(package_name: str, *paths: str) -> str:
    return os.path.join(get_package_share_directory(package_name), *paths)


def _load_go2_robot_description() -> str:
    root = ET.parse(_share('go2_robot_sdk', 'urdf', 'go2.urdf')).getroot()
    for element in list(root):
        if element.tag == 'link' and element.attrib.get('name') in {'map', 'odom'}:
            root.remove(element)
        elif element.tag == 'joint' and element.attrib.get('name') in {'map_joint', 'odom_joint'}:
            root.remove(element)
        elif element.tag == 'link' and element.attrib.get('name') == 'base_link':
            inertial = element.find('inertial')
            if inertial is not None:
                element.remove(inertial)
    return ET.tostring(root, encoding='unicode')


def generate_launch_description():
    extrinsics_file = LaunchConfiguration('extrinsics_file')
    robot_ip = LaunchConfiguration('robot_ip')
    robot_token = LaunchConfiguration('robot_token')
    conn_type = LaunchConfiguration('conn_type')
    enable_video = LaunchConfiguration('enable_video')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_lateral_speed = LaunchConfiguration('max_lateral_speed')
    max_yaw_rate = LaunchConfiguration('max_yaw_rate')
    autonomy_speed = LaunchConfiguration('autonomy_speed')
    rviz = LaunchConfiguration('rviz')
    keyboard_teleop = LaunchConfiguration('keyboard_teleop')

    robot_description = _load_go2_robot_description()
    rviz_config = _share('go2_bridge', 'config', 'go2_aede_real.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'extrinsics_file',
            default_value=_share('go2_bridge', 'config', 'extrinsics.yaml'),
            description='Static TF/extrinsics parameter file'),
        DeclareLaunchArgument(
            'robot_ip',
            default_value=EnvironmentVariable(
                'ROBOT_IP',
                default_value=EnvironmentVariable('GO2_IP', default_value='')),
            description='Go2 robot IP address'),
        DeclareLaunchArgument(
            'robot_token',
            default_value=EnvironmentVariable(
                'ROBOT_TOKEN',
                default_value=EnvironmentVariable('GO2_TOKEN', default_value='')),
            description='Go2 robot token'),
        DeclareLaunchArgument(
            'conn_type',
            default_value='webrtc',
            description='Go2 SDK connection type'),
        DeclareLaunchArgument(
            'enable_video',
            default_value='false',
            description='Enable Go2 WebRTC video stream'),
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='0.5',
            description='Clamp forwarded Go2 linear x speed in m/s'),
        DeclareLaunchArgument(
            'max_lateral_speed',
            default_value='0.3',
            description='Clamp forwarded Go2 linear y speed in m/s'),
        DeclareLaunchArgument(
            'max_yaw_rate',
            default_value='0.8',
            description='Clamp forwarded Go2 yaw rate in rad/s'),
        DeclareLaunchArgument(
            'autonomy_speed',
            default_value='0.3',
            description='AEDE autonomy target speed in m/s'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz with Go2 + AEDE config'),
        DeclareLaunchArgument(
            'keyboard_teleop',
            default_value='false',
            description='Launch keyboard teleop; requires a real TTY to control'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='go2_robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 50.0,
            }],
        ),

        Node(
            package='go2_bridge',
            executable='go2_sdk_driver_node',
            name='go2_driver_node',
            output='screen',
            remappings=[
                ('joint_states', '/go2/joint_states_raw'),
            ],
            parameters=[{
                'robot_ip': robot_ip,
                'token': robot_token,
                'conn_type': conn_type,
                'decode_lidar': True,
                'enable_video': enable_video,
                'publish_raw_voxel': False,
                'obstacle_avoidance': False,
            }],
        ),

        Node(
            package='go2_bridge',
            executable='joint_state_republisher',
            name='go2_joint_state_republisher',
            output='screen',
            parameters=[{
                'input_topic': '/go2/joint_states_raw',
                'output_topic': '/joint_states',
                'publish_rate': 20.0,
            }],
        ),

        Node(
            package='go2_bridge',
            executable='go2_bridge_node',
            name='go2_aede_bridge',
            output='screen',
            parameters=[extrinsics_file, {
                'max_linear_speed': max_linear_speed,
                'max_lateral_speed': max_lateral_speed,
                'max_yaw_rate': max_yaw_rate,
            }],
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                _share('terrain_analysis', 'launch', 'terrain_analysis.launch')),
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                _share('terrain_analysis_ext', 'launch', 'terrain_analysis_ext.launch')),
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                _share('sensor_scan_generation', 'launch', 'sensor_scan_generation.launch')),
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                _share('local_planner', 'launch', 'local_planner.launch')),
            launch_arguments={
                'maxSpeed': max_linear_speed,
                'autonomySpeed': autonomy_speed,
                'autonomyMode': 'true',
            }.items(),
        ),

        Node(
            package='go2_bridge',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            output='screen',
            condition=IfCondition(keyboard_teleop),
            parameters=[{
                'max_speed': max_linear_speed,
                'max_yaw_rate': max_yaw_rate,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
