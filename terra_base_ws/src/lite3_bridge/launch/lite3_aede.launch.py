#!/usr/bin/env python3
"""Launch Lite3 topic bridge with the core AEDE stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    input_profile = LaunchConfiguration('input_profile')
    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    enable_motion = LaunchConfiguration('enable_motion')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_yaw_rate = LaunchConfiguration('max_yaw_rate')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    rewrite_scan_frame = LaunchConfiguration('rewrite_scan_frame')
    max_speed = LaunchConfiguration('maxSpeed')
    autonomy_speed = LaunchConfiguration('autonomySpeed')
    vehicle_length = LaunchConfiguration('vehicleLength')
    vehicle_width = LaunchConfiguration('vehicleWidth')
    sensor_offset_x = LaunchConfiguration('sensorOffsetX')
    sensor_offset_y = LaunchConfiguration('sensorOffsetY')
    sensor_offset_z = LaunchConfiguration('sensorOffsetZ')
    sensor_roll = LaunchConfiguration('sensorRoll')
    sensor_pitch = LaunchConfiguration('sensorPitch')
    sensor_yaw = LaunchConfiguration('sensorYaw')
    input_to_vehicle_x = LaunchConfiguration('inputToVehicleX')
    input_to_vehicle_y = LaunchConfiguration('inputToVehicleY')
    input_to_vehicle_z = LaunchConfiguration('inputToVehicleZ')
    input_to_vehicle_roll = LaunchConfiguration('inputToVehicleRoll')
    input_to_vehicle_pitch = LaunchConfiguration('inputToVehiclePitch')
    input_to_vehicle_yaw = LaunchConfiguration('inputToVehicleYaw')
    camera_offset_z = LaunchConfiguration('cameraOffsetZ')
    local_planner_paths = PathJoinSubstitution([
        FindPackageShare('local_planner'),
        'paths',
    ])

    enable_motion_param = ParameterValue(enable_motion, value_type=bool)
    max_linear_speed_param = ParameterValue(
        max_linear_speed, value_type=float)
    max_yaw_rate_param = ParameterValue(max_yaw_rate, value_type=float)
    rewrite_scan_frame_param = ParameterValue(
        rewrite_scan_frame, value_type=bool)
    max_speed_param = ParameterValue(max_speed, value_type=float)
    autonomy_speed_param = ParameterValue(autonomy_speed, value_type=float)
    vehicle_length_param = ParameterValue(vehicle_length, value_type=float)
    vehicle_width_param = ParameterValue(vehicle_width, value_type=float)
    sensor_offset_x_param = ParameterValue(sensor_offset_x, value_type=float)
    sensor_offset_y_param = ParameterValue(sensor_offset_y, value_type=float)
    sensor_offset_z_param = ParameterValue(sensor_offset_z, value_type=float)
    sensor_roll_param = ParameterValue(sensor_roll, value_type=float)
    sensor_pitch_param = ParameterValue(sensor_pitch, value_type=float)
    sensor_yaw_param = ParameterValue(sensor_yaw, value_type=float)
    input_to_vehicle_x_param = ParameterValue(
        input_to_vehicle_x, value_type=float)
    input_to_vehicle_y_param = ParameterValue(
        input_to_vehicle_y, value_type=float)
    input_to_vehicle_z_param = ParameterValue(
        input_to_vehicle_z, value_type=float)
    input_to_vehicle_roll_param = ParameterValue(
        input_to_vehicle_roll, value_type=float)
    input_to_vehicle_pitch_param = ParameterValue(
        input_to_vehicle_pitch, value_type=float)
    input_to_vehicle_yaw_param = ParameterValue(
        input_to_vehicle_yaw, value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('input_profile', default_value='raw_smoke'),
        DeclareLaunchArgument('odom_topic', default_value=''),
        DeclareLaunchArgument('scan_topic', default_value=''),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.2'),
        DeclareLaunchArgument('max_yaw_rate', default_value='0.3'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('rewrite_scan_frame', default_value='true'),
        DeclareLaunchArgument('maxSpeed', default_value='0.2'),
        DeclareLaunchArgument('autonomySpeed', default_value='0.2'),
        DeclareLaunchArgument('vehicleLength', default_value='0.6'),
        DeclareLaunchArgument('vehicleWidth', default_value='0.45'),
        DeclareLaunchArgument('sensorOffsetX', default_value='0.0'),
        DeclareLaunchArgument('sensorOffsetY', default_value='0.0'),
        DeclareLaunchArgument('sensorOffsetZ', default_value='0.0'),
        DeclareLaunchArgument('sensorRoll', default_value='0.0'),
        DeclareLaunchArgument('sensorPitch', default_value='0.0'),
        DeclareLaunchArgument('sensorYaw', default_value='0.0'),
        DeclareLaunchArgument('inputToVehicleX', default_value='0.0'),
        DeclareLaunchArgument('inputToVehicleY', default_value='0.0'),
        DeclareLaunchArgument('inputToVehicleZ', default_value='0.0'),
        DeclareLaunchArgument('inputToVehicleRoll', default_value='0.0'),
        DeclareLaunchArgument('inputToVehiclePitch', default_value='0.0'),
        DeclareLaunchArgument('inputToVehicleYaw', default_value='0.0'),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0.0'),

        Node(
            package='lite3_bridge',
            executable='lite3_bridge_node',
            name='lite3_aede_bridge',
            output='screen',
            parameters=[{
                'input_profile': input_profile,
                'odom_topic': odom_topic,
                'scan_topic': scan_topic,
                'enable_motion': enable_motion_param,
                'max_linear_speed': max_linear_speed_param,
                'max_yaw_rate': max_yaw_rate_param,
                'cmd_vel_topic': cmd_vel_topic,
                'rewrite_scan_frame': rewrite_scan_frame_param,
                'sensor_x': sensor_offset_x_param,
                'sensor_y': sensor_offset_y_param,
                'sensor_z': sensor_offset_z_param,
                'sensor_roll': sensor_roll_param,
                'sensor_pitch': sensor_pitch_param,
                'sensor_yaw': sensor_yaw_param,
                'input_to_vehicle_x': input_to_vehicle_x_param,
                'input_to_vehicle_y': input_to_vehicle_y_param,
                'input_to_vehicle_z': input_to_vehicle_z_param,
                'input_to_vehicle_roll': input_to_vehicle_roll_param,
                'input_to_vehicle_pitch': input_to_vehicle_pitch_param,
                'input_to_vehicle_yaw': input_to_vehicle_yaw_param,
            }],
        ),

        Node(
            package='terrain_analysis',
            executable='terrainAnalysis',
            name='terrainAnalysis',
            output='screen',
            parameters=[{
                'scanVoxelSize': 0.05,
                'decayTime': 2.0,
                'noDecayDis': 4.0,
                'clearingDis': 8.0,
                'useSorting': False,
                'quantileZ': 0.25,
                'considerDrop': True,
                'limitGroundLift': False,
                'maxGroundLift': 0.15,
                'clearDyObs': True,
                'minDyObsDis': 0.3,
                'minDyObsAngle': 0.0,
                'minDyObsRelZ': -0.5,
                'absDyObsRelZThre': 0.2,
                'minDyObsVFOV': -16.0,
                'maxDyObsVFOV': 16.0,
                'minDyObsPointNum': 1,
                'noDataObstacle': False,
                'noDataBlockSkipNum': 0,
                'minBlockPointNum': 10,
                'vehicleHeight': 1.5,
                'voxelPointUpdateThre': 100,
                'voxelTimeUpdateThre': 2.0,
                'minRelZ': -2.5,
                'maxRelZ': 1.0,
                'disRatioZ': 0.2,
            }],
        ),

        Node(
            package='terrain_analysis_ext',
            executable='terrainAnalysisExt',
            name='terrainAnalysisExt',
            output='screen',
            parameters=[{
                'scanVoxelSize': 0.1,
                'decayTime': 10.0,
                'noDecayDis': 0.0,
                'clearingDis': 30.0,
                'useSorting': False,
                'quantileZ': 0.1,
                'vehicleHeight': 1.5,
                'voxelPointUpdateThre': 100,
                'voxelTimeUpdateThre': 2.0,
                'lowerBoundZ': -2.5,
                'upperBoundZ': 1.0,
                'disRatioZ': 0.1,
                'checkTerrainConn': False,
                'terrainConnThre': 0.5,
                'terrainUnderVehicle': -0.75,
                'ceilingFilteringThre': 2.0,
                'localTerrainMapRadius': 4.0,
            }],
        ),

        Node(
            package='sensor_scan_generation',
            executable='sensorScanGeneration',
            name='sensorScanGeneration',
            output='screen',
        ),

        Node(
            package='local_planner',
            executable='localPlanner',
            name='localPlanner',
            output='screen',
            remappings=[
                ('/path', '/aede/path'),
                ('/free_paths', '/aede/free_paths'),
            ],
            parameters=[{
                'pathFolder': local_planner_paths,
                'vehicleLength': vehicle_length_param,
                'vehicleWidth': vehicle_width_param,
                'sensorOffsetX': sensor_offset_x_param,
                'sensorOffsetY': sensor_offset_y_param,
                'twoWayDrive': True,
                'laserVoxelSize': 0.05,
                'terrainVoxelSize': 0.2,
                'useTerrainAnalysis': True,
                'checkObstacle': True,
                'checkRotObstacle': False,
                'adjacentRange': 4.25,
                'obstacleHeightThre': 0.15,
                'groundHeightThre': 0.1,
                'costHeightThre': 0.1,
                'costScore': 0.02,
                'useCost': False,
                'pointPerPathThre': 2,
                'minRelZ': -0.5,
                'maxRelZ': 0.25,
                'maxSpeed': max_speed_param,
                'dirWeight': 0.02,
                'dirThre': 90.0,
                'dirToVehicle': False,
                'pathScale': 1.25,
                'minPathScale': 0.75,
                'pathScaleStep': 0.25,
                'pathScaleBySpeed': True,
                'minPathRange': 1.0,
                'pathRangeStep': 0.5,
                'pathRangeBySpeed': True,
                'pathCropByGoal': True,
                'autonomyMode': True,
                'autonomySpeed': autonomy_speed_param,
                'joyToSpeedDelay': 2.0,
                'joyToCheckObstacleDelay': 5.0,
                'goalClearRange': 0.5,
                'goalX': 0.0,
                'goalY': 0.0,
            }],
        ),

        Node(
            package='local_planner',
            executable='pathFollower',
            name='pathFollower',
            output='screen',
            remappings=[
                ('/path', '/aede/path'),
                ('/cmd_vel', '/aede/cmd_vel_stamped'),
            ],
            parameters=[{
                'sensorOffsetX': sensor_offset_x_param,
                'sensorOffsetY': sensor_offset_y_param,
                'pubSkipNum': 1,
                'twoWayDrive': True,
                'lookAheadDis': 0.5,
                'yawRateGain': 7.5,
                'stopYawRateGain': 7.5,
                'maxYawRate': 90.0,
                'maxSpeed': max_speed_param,
                'maxAccel': 2.5,
                'switchTimeThre': 1.0,
                'dirDiffThre': 0.1,
                'stopDisThre': 0.2,
                'slowDwnDisThre': 0.85,
                'useInclRateToSlow': False,
                'inclRateThre': 120.0,
                'slowRate1': 0.25,
                'slowRate2': 0.5,
                'slowTime1': 2.0,
                'slowTime2': 2.0,
                'useInclToStop': False,
                'inclThre': 45.0,
                'stopTime': 5.0,
                'noRotAtStop': False,
                'noRotAtGoal': True,
                'autonomyMode': True,
                'autonomySpeed': autonomy_speed_param,
                'joyToSpeedDelay': 2.0,
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensorTransPublisher',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', camera_offset_z,
                '--roll', '-1.5707963',
                '--pitch', '0',
                '--yaw', '-1.5707963',
                '--frame-id', 'sensor',
                '--child-frame-id', 'camera',
            ],
        ),
    ])
