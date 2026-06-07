#!/usr/bin/env python3
"""
Launch file for Go2 robot with the original SDK and AEDE bridge.

Launches:
  1. go2_driver_node  - WebRTC connection, data publishing
  2. go2_aede_bridge  - TF/topic conversion for AEDE
  3. keyboard_teleop  - keyboard control (default: enabled)

Usage:
  # With keyboard teleop (default):
  ROBOT_IP=192.168.1.100 ROBOT_TOKEN=abc123 \
    ros2 launch go2_bridge robot_bringup.launch.py

  # Or pass params explicitly:
  ros2 launch go2_bridge robot_bringup.launch.py \
    robot_ip:=192.168.1.100 robot_token:=abc123

  # Without keyboard teleop (for AEDE autonomous mode):
  ros2 launch go2_bridge robot_bringup.launch.py teleop:=false
"""

import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def _share(package_name: str, *paths: str) -> str:
    return os.path.join(get_package_share_directory(package_name), *paths)


def _load_go2_robot_description() -> str:
    urdf_path = _share('go2_robot_sdk', 'urdf', 'go2.urdf')
    root = ET.parse(urdf_path).getroot()
    for element in list(root):
        if element.tag == 'link' and element.attrib.get('name') in {'map', 'odom'}:
            root.remove(element)
        elif element.tag == 'joint' and element.attrib.get('name') in {'map_joint', 'odom_joint'}:
            root.remove(element)
    return ET.tostring(root, encoding='unicode')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'extrinsics_file',
            default_value=_share('go2_bridge', 'config', 'extrinsics.yaml'),
            description='Static TF/extrinsics parameter file'),
        DeclareLaunchArgument(
            'robot_ip',
            default_value=EnvironmentVariable('ROBOT_IP',
                             default_value=EnvironmentVariable('GO2_IP',
                                              default_value='')),
            description='Go2 robot IP address'),
        DeclareLaunchArgument(
            'robot_token',
            default_value=EnvironmentVariable('ROBOT_TOKEN',
                             default_value=EnvironmentVariable('GO2_TOKEN',
                                              default_value='')),
            description='Go2 robot token'),
        DeclareLaunchArgument(
            'conn_type', default_value='webrtc',
            description='Connection type (webrtc)'),
        DeclareLaunchArgument(
            'teleop', default_value='true',
            description='Enable keyboard teleoperation'),
        DeclareLaunchArgument(
            'enable_video', default_value='false',
            description='Enable mono camera video stream'),
        DeclareLaunchArgument(
            'max_speed', default_value='0.7',
            description='Max linear speed for teleop (m/s)'),

        # ---- Robot State Publisher (URDF TF + /robot_description) ----
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': _load_go2_robot_description(),
                'publish_frequency': 50.0,
            }],
        ),

        # ---- Go2 Driver Node ----
        Node(
            package='go2_bridge',
            executable='go2_sdk_driver_node',
            name='go2_driver_node',
            output='screen',
            remappings=[
                ('joint_states', '/go2/joint_states_raw'),
            ],
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'token': LaunchConfiguration('robot_token'),
                'conn_type': LaunchConfiguration('conn_type'),
                'decode_lidar': True,
                'enable_video': LaunchConfiguration('enable_video'),
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

        # ---- AEDE Bridge Node ----
        Node(
            package='go2_bridge',
            executable='go2_bridge_node',
            name='go2_aede_bridge',
            output='screen',
            parameters=[LaunchConfiguration('extrinsics_file')],
        ),

        # ---- Keyboard Teleop (optional) ----
        Node(
            package='go2_bridge',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            output='screen',
            condition=IfCondition(LaunchConfiguration('teleop')),
            parameters=[{
                'max_speed': LaunchConfiguration('max_speed'),
                'max_yaw_rate': 1.0,
            }],
        ),
    ])
