#!/usr/bin/env python3
"""Bridge Lite3 ROS topics into AEDE's real-robot topic interface."""

import math
from copy import deepcopy

import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


PROFILE_DEFAULTS = {
    'raw_smoke': {
        'odom_topic': '/leg_odom2',
        'scan_topic': '/rslidar_points',
    },
    'faster_lio': {
        'odom_topic': '/Odometry',
        'scan_topic': '/cloud_registered',
    },
}


class Lite3AedeBridge(Node):
    """Republish Lite3 odometry/clouds and gate AEDE velocity commands."""

    def __init__(self):
        super().__init__('lite3_aede_bridge')

        self.declare_parameter('input_profile', 'raw_smoke')
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('scan_topic', '')
        self.declare_parameter('state_estimation_topic', '/state_estimation')
        self.declare_parameter('registered_scan_topic', '/registered_scan')
        self.declare_parameter('aede_cmd_topic', '/aede/cmd_vel_stamped')
        self.declare_parameter('cmd_vel_preview_topic', '/cmd_vel_preview')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('max_linear_speed', 0.2)
        self.declare_parameter('max_yaw_rate', 0.3)
        self.declare_parameter('rewrite_scan_frame', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('sensor_frame', 'sensor')
        self.declare_parameter('vehicle_frame', 'vehicle')
        self.declare_parameter('sensor_x', 0.0)
        self.declare_parameter('sensor_y', 0.0)
        self.declare_parameter('sensor_z', 0.0)
        self.declare_parameter('sensor_roll', 0.0)
        self.declare_parameter('sensor_pitch', 0.0)
        self.declare_parameter('sensor_yaw', 0.0)
        self.declare_parameter('input_to_vehicle_x', 0.0)
        self.declare_parameter('input_to_vehicle_y', 0.0)
        self.declare_parameter('input_to_vehicle_z', 0.0)
        self.declare_parameter('input_to_vehicle_roll', 0.0)
        self.declare_parameter('input_to_vehicle_pitch', 0.0)
        self.declare_parameter('input_to_vehicle_yaw', 0.0)

        profile = str(self.get_parameter('input_profile').value)
        if profile not in PROFILE_DEFAULTS:
            raise ValueError(
                f"input_profile must be one of {sorted(PROFILE_DEFAULTS)}, got {profile!r}"
            )

        profile_defaults = PROFILE_DEFAULTS[profile]
        self.odom_topic = self._string_param(
            'odom_topic', profile_defaults['odom_topic'])
        self.scan_topic = self._string_param(
            'scan_topic', profile_defaults['scan_topic'])
        self.state_estimation_topic = self._string_param(
            'state_estimation_topic', '/state_estimation')
        self.registered_scan_topic = self._string_param(
            'registered_scan_topic', '/registered_scan')
        self.aede_cmd_topic = self._string_param(
            'aede_cmd_topic', '/aede/cmd_vel_stamped')
        self.cmd_vel_preview_topic = self._string_param(
            'cmd_vel_preview_topic', '/cmd_vel_preview')
        self.cmd_vel_topic = self._string_param('cmd_vel_topic', '/cmd_vel')
        self.enable_motion = bool(self.get_parameter('enable_motion').value)
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.rewrite_scan_frame = bool(
            self.get_parameter('rewrite_scan_frame').value)

        self.map_frame = self._string_param('map_frame', 'map')
        self.sensor_frame = self._string_param('sensor_frame', 'sensor')
        self.vehicle_frame = self._string_param('vehicle_frame', 'vehicle')
        self.sensor_offset = (
            float(self.get_parameter('sensor_x').value),
            float(self.get_parameter('sensor_y').value),
            float(self.get_parameter('sensor_z').value),
        )
        self.sensor_rotation = self._euler_to_quaternion(
            float(self.get_parameter('sensor_roll').value),
            float(self.get_parameter('sensor_pitch').value),
            float(self.get_parameter('sensor_yaw').value),
        )
        self.input_to_vehicle_offset = (
            float(self.get_parameter('input_to_vehicle_x').value),
            float(self.get_parameter('input_to_vehicle_y').value),
            float(self.get_parameter('input_to_vehicle_z').value),
        )
        self.input_to_vehicle_rotation = self._euler_to_quaternion(
            float(self.get_parameter('input_to_vehicle_roll').value),
            float(self.get_parameter('input_to_vehicle_pitch').value),
            float(self.get_parameter('input_to_vehicle_yaw').value),
        )

        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(
            self._sensor_to_vehicle_transform())

        self.state_estimation_pub = self.create_publisher(
            Odometry, self.state_estimation_topic, reliable_qos)
        self.registered_scan_pub = self.create_publisher(
            PointCloud2, self.registered_scan_topic, reliable_qos)
        self.cmd_preview_pub = self.create_publisher(
            Twist, self.cmd_vel_preview_topic, reliable_qos)
        self.cmd_vel_pub = None
        if self.enable_motion:
            self.cmd_vel_pub = self.create_publisher(
                Twist, self.cmd_vel_topic, reliable_qos)

        self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, sensor_qos)
        self.create_subscription(
            PointCloud2, self.scan_topic, self._on_scan, sensor_qos)
        self.create_subscription(
            TwistStamped, self.aede_cmd_topic, self._on_cmd, reliable_qos)

        motion_state = 'enabled' if self.enable_motion else 'disabled'
        self.get_logger().info(
            f"Lite3-AEDE bridge started with profile={profile}, "
            f"odom={self.odom_topic}, scan={self.scan_topic}, "
            f"motion={motion_state}"
        )

    def _string_param(self, name: str, default: str) -> str:
        value = str(self.get_parameter(name).value)
        return value if value else default

    def _sensor_to_vehicle_transform(self) -> TransformStamped:
        inverse_rotation = self._normalize_quaternion(
            self._quaternion_conjugate(self.sensor_rotation))
        inverse_translation = self._rotate_vector(
            inverse_rotation,
            tuple(-value for value in self.sensor_offset),
        )

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.sensor_frame
        transform.child_frame_id = self.vehicle_frame
        transform.transform.translation.x = inverse_translation[0]
        transform.transform.translation.y = inverse_translation[1]
        transform.transform.translation.z = inverse_translation[2]
        transform.transform.rotation.x = inverse_rotation[0]
        transform.transform.rotation.y = inverse_rotation[1]
        transform.transform.rotation.z = inverse_rotation[2]
        transform.transform.rotation.w = inverse_rotation[3]
        return transform

    def _on_odom(self, msg: Odometry) -> None:
        input_position = msg.pose.pose.position
        input_orientation = msg.pose.pose.orientation
        input_quaternion = self._normalize_quaternion((
            input_orientation.x,
            input_orientation.y,
            input_orientation.z,
            input_orientation.w,
        ))
        vehicle_position, vehicle_quaternion = self._compose_transform(
            (
                input_position.x,
                input_position.y,
                input_position.z,
            ),
            input_quaternion,
            self.input_to_vehicle_offset,
            self.input_to_vehicle_rotation,
        )
        sensor_position, sensor_quaternion = self._compose_transform(
            vehicle_position,
            vehicle_quaternion,
            self.sensor_offset,
            self.sensor_rotation,
        )

        state = Odometry()
        state.header.stamp = msg.header.stamp
        state.header.frame_id = self.map_frame
        state.child_frame_id = self.sensor_frame
        state.pose = deepcopy(msg.pose)
        state.twist = deepcopy(msg.twist)
        state.pose.pose.position.x = sensor_position[0]
        state.pose.pose.position.y = sensor_position[1]
        state.pose.pose.position.z = sensor_position[2]
        state.pose.pose.orientation.x = sensor_quaternion[0]
        state.pose.pose.orientation.y = sensor_quaternion[1]
        state.pose.pose.orientation.z = sensor_quaternion[2]
        state.pose.pose.orientation.w = sensor_quaternion[3]

        self.state_estimation_pub.publish(state)
        self.tf_broadcaster.sendTransform(
            self._state_to_transform(state, self.sensor_frame))

    def _on_scan(self, msg: PointCloud2) -> None:
        scan = deepcopy(msg)
        if self.rewrite_scan_frame:
            scan.header.frame_id = self.map_frame
        self.registered_scan_pub.publish(scan)

    def _on_cmd(self, msg: TwistStamped) -> None:
        twist = Twist()
        twist.linear.x = self._clamp(
            msg.twist.linear.x, -self.max_linear_speed, self.max_linear_speed)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self._clamp(
            msg.twist.angular.z, -self.max_yaw_rate, self.max_yaw_rate)

        self.cmd_preview_pub.publish(twist)
        if self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(twist)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, float(value)))

    @staticmethod
    def _state_to_transform(state: Odometry, child_frame: str) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = state.header.stamp
        transform.header.frame_id = state.header.frame_id
        transform.child_frame_id = child_frame
        transform.transform.translation.x = state.pose.pose.position.x
        transform.transform.translation.y = state.pose.pose.position.y
        transform.transform.translation.z = state.pose.pose.position.z
        transform.transform.rotation = state.pose.pose.orientation
        return transform

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    @staticmethod
    def _quaternion_multiply(first, second):
        x1, y1, z1, w1 = first
        x2, y2, z2, w2 = second
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    @staticmethod
    def _quaternion_conjugate(quaternion):
        x_value, y_value, z_value, w_value = quaternion
        return (-x_value, -y_value, -z_value, w_value)

    @classmethod
    def _rotate_vector(cls, quaternion, vector):
        vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
        rotated = cls._quaternion_multiply(
            cls._quaternion_multiply(quaternion, vector_quaternion),
            cls._quaternion_conjugate(quaternion),
        )
        return rotated[:3]

    @classmethod
    def _compose_transform(cls, parent_position, parent_quaternion,
                           child_offset, child_rotation):
        parent_quaternion = cls._normalize_quaternion(parent_quaternion)
        child_rotation = cls._normalize_quaternion(child_rotation)
        rotated_offset = cls._rotate_vector(parent_quaternion, child_offset)
        child_position = (
            parent_position[0] + rotated_offset[0],
            parent_position[1] + rotated_offset[1],
            parent_position[2] + rotated_offset[2],
        )
        child_quaternion = cls._normalize_quaternion(
            cls._quaternion_multiply(parent_quaternion, child_rotation)
        )
        return child_position, child_quaternion

    @staticmethod
    def _normalize_quaternion(quaternion):
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm == 0.0:
            return (0.0, 0.0, 0.0, 1.0)
        return tuple(value / norm for value in quaternion)


def main():
    rclpy.init()
    node = Lite3AedeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
