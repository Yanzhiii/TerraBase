#!/usr/bin/env python3
"""Real Go2 + ZED 2i + STEPP + AEDE local planner bringup."""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _share(package_name: str, *paths: str) -> str:
    return os.path.join(get_package_share_directory(package_name), *paths)


def _find_stepp_python_root() -> str:
    current = os.path.abspath(_share('go2_bridge'))
    for _ in range(8):
        candidate = os.path.join(current, 'third_party', 'STEPP-Code')
        if os.path.isdir(os.path.join(candidate, 'STEPP')):
            return candidate
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent
    return ''


def _find_stepp_model_path() -> str:
    env_path = os.environ.get('STEPP_MODEL_PATH', '')
    if env_path and os.path.isfile(os.path.expanduser(env_path)):
        return os.path.expanduser(env_path)

    stepp_root = _find_stepp_python_root()
    if not stepp_root:
        return ''

    candidate = os.path.join(
        stepp_root,
        'checkpoints',
        'all_ViT_small_input_700_big_nn_checkpoint_20240827-1935.pth',
    )
    return candidate if os.path.isfile(candidate) else ''


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
    config_file = LaunchConfiguration('config_file')
    extrinsics_file = LaunchConfiguration('extrinsics_file')
    go2_bridge_share = _share('go2_bridge')
    robot_ip = LaunchConfiguration('robot_ip')
    robot_token = LaunchConfiguration('robot_token')
    conn_type = LaunchConfiguration('conn_type')
    enable_video = LaunchConfiguration('enable_video')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_lateral_speed = LaunchConfiguration('max_lateral_speed')
    max_yaw_rate = LaunchConfiguration('max_yaw_rate')
    planner_max_yaw_rate_deg = LaunchConfiguration('planner_max_yaw_rate_deg')
    autonomy_speed = LaunchConfiguration('autonomy_speed')
    rviz = LaunchConfiguration('rviz')
    keyboard_teleop = LaunchConfiguration('keyboard_teleop')

    zed_camera_name = LaunchConfiguration('zed_camera_name')
    zed_camera_model = LaunchConfiguration('zed_camera_model')
    zed_serial_number = LaunchConfiguration('zed_serial_number')
    zed_publish_tf = LaunchConfiguration('zed_publish_tf')
    zed_publish_map_tf = LaunchConfiguration('zed_publish_map_tf')
    zed_publish_urdf = LaunchConfiguration('zed_publish_urdf')
    zed_enable_ipc = LaunchConfiguration('zed_enable_ipc')
    zed_depth_mode = LaunchConfiguration('zed_depth_mode')
    zed_param_overrides = LaunchConfiguration('zed_param_overrides')

    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    stepp_model_path = LaunchConfiguration('stepp_model_path')
    stepp_visualize = LaunchConfiguration('stepp_visualize')
    stepp_ump = LaunchConfiguration('stepp_ump')
    stepp_cutoff = LaunchConfiguration('stepp_cutoff')
    stepp_python_root = LaunchConfiguration('stepp_python_root')

    robot_description = _load_go2_robot_description()
    rviz_config = _share('go2_bridge', 'config', 'go2_aede_real.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=_share('go2_bridge', 'config', 'go2_stepp_aede.yaml'),
            description='TerraBase parameter file for Go2 + ZED + STEPP + AEDE'),
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
            default_value='0.3',
            description='Clamp forwarded Go2 linear x speed in m/s'),
        DeclareLaunchArgument(
            'max_lateral_speed',
            default_value='0.2',
            description='Clamp forwarded Go2 linear y speed in m/s'),
        DeclareLaunchArgument(
            'max_yaw_rate',
            default_value='0.5',
            description='Clamp forwarded Go2 yaw rate in rad/s'),
        DeclareLaunchArgument(
            'planner_max_yaw_rate_deg',
            default_value='28.6478897565',
            description='AEDE pathFollower yaw-rate limit in deg/s'),
        DeclareLaunchArgument(
            'autonomy_speed',
            default_value='0.2',
            description='AEDE autonomy target speed in m/s'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz with Go2 + AEDE config'),
        DeclareLaunchArgument(
            'keyboard_teleop',
            default_value='false',
            description='Launch keyboard teleop'),

        DeclareLaunchArgument(
            'zed_camera_name',
            default_value='zed',
            description='ZED camera namespace/name'),
        DeclareLaunchArgument(
            'zed_camera_model',
            default_value='zed2i',
            description='ZED camera model'),
        DeclareLaunchArgument(
            'zed_serial_number',
            default_value=EnvironmentVariable('ZED_SERIAL_NUMBER', default_value='0'),
            description='ZED serial number, or 0 for the first camera'),
        DeclareLaunchArgument(
            'zed_publish_tf',
            default_value='false',
            description='Keep false; TerraBase publishes calibrated camera TF'),
        DeclareLaunchArgument(
            'zed_publish_map_tf',
            default_value='false',
            description='Keep false; TerraBase/Go2 owns map and odom frames'),
        DeclareLaunchArgument(
            'zed_publish_urdf',
            default_value='true',
            description='Publish ZED camera URDF links for visualization'),
        DeclareLaunchArgument(
            'zed_enable_ipc',
            default_value='true',
            description='Enable official zed_wrapper intra-process communication'),
        DeclareLaunchArgument(
            'zed_depth_mode',
            default_value='NEURAL_LIGHT',
            description='ZED SDK depth mode used by zed_wrapper'),
        DeclareLaunchArgument(
            'zed_param_overrides',
            default_value='',
            description='Extra zed_wrapper inline overrides, semicolon-separated key:=value pairs'),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/zed/zed_node/rgb/color/rect/image/compressed',
            description='Compressed RGB topic consumed by STEPP inference'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/zed/zed_node/depth/depth_registered',
            description='Registered depth topic consumed by TerraBase projector'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/zed/zed_node/rgb/color/rect/camera_info',
            description='CameraInfo topic used for dynamic intrinsics'),
        DeclareLaunchArgument(
            'stepp_model_path',
            default_value=_find_stepp_model_path(),
            description='Path to the STEPP .pth checkpoint'),
        DeclareLaunchArgument(
            'stepp_visualize',
            default_value='true',
            description='Publish STEPP overlay and segmentation debug images'),
        DeclareLaunchArgument(
            'stepp_ump',
            default_value='false',
            description='Use STEPP mixed precision inference'),
        DeclareLaunchArgument(
            'stepp_cutoff',
            default_value='0.45',
            description='STEPP normalized reconstruction-error cutoff'),
        DeclareLaunchArgument(
            'stepp_python_root',
            default_value=_find_stepp_python_root(),
            description='Path to STEPP-Code root used for STEPP Python imports'),

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _share('zed_wrapper', 'launch', 'zed_camera.launch.py')),
            launch_arguments={
                'camera_name': zed_camera_name,
                'camera_model': zed_camera_model,
                'serial_number': zed_serial_number,
                'publish_tf': zed_publish_tf,
                'publish_map_tf': zed_publish_map_tf,
                'publish_urdf': zed_publish_urdf,
                'enable_ipc': zed_enable_ipc,
                'param_overrides': [
                    'depth.depth_mode:=',
                    zed_depth_mode,
                    ';',
                    zed_param_overrides,
                ],
            }.items(),
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
            parameters=[config_file, extrinsics_file, {
                'max_linear_speed': max_linear_speed,
                'max_lateral_speed': max_lateral_speed,
                'max_yaw_rate': max_yaw_rate,
            }],
        ),

        Node(
            package='stepp_ros2_humble',
            executable='inference_node.py',
            name='inference_node',
            output='screen',
            parameters=[{
                'model_path': stepp_model_path,
                'visualize': stepp_visualize,
                'ump': stepp_ump,
                'cutoff': stepp_cutoff,
            }],
            remappings=[
                ('/camera/color/image_raw/compressed', rgb_topic),
            ],
            additional_env={
                'PYTHONPATH': [
                    os.path.join(go2_bridge_share, 'python_site'),
                    ':',
                    stepp_python_root,
                    ':',
                    EnvironmentVariable('PYTHONPATH', default_value=''),
                ],
            },
        ),

        Node(
            package='go2_bridge',
            executable='stepp_depth_projector',
            name='stepp_depth_projector',
            output='screen',
            parameters=[config_file, extrinsics_file],
            remappings=[
                ('/camera/color/camera_info', camera_info_topic),
                ('/camera/aligned_depth_to_color/image_raw', depth_topic),
                ('/depth_projection', '/terrain_map'),
            ],
        ),

        Node(
            package='local_planner',
            executable='localPlanner',
            name='localPlanner',
            output='screen',
            parameters=[config_file, extrinsics_file, {
                'pathFolder': _share('local_planner', 'paths'),
                'maxSpeed': max_linear_speed,
                'autonomySpeed': autonomy_speed,
                'autonomyMode': True,
            }],
        ),

        Node(
            package='local_planner',
            executable='pathFollower',
            name='pathFollower',
            output='screen',
            parameters=[config_file, extrinsics_file, {
                'maxSpeed': max_linear_speed,
                'maxYawRate': planner_max_yaw_rate_deg,
                'autonomySpeed': autonomy_speed,
                'autonomyMode': True,
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='vehicleTransPublisher',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id',
                'sensor',
                '--child-frame-id',
                'vehicle',
            ],
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
