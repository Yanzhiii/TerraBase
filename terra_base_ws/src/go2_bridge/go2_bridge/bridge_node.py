#!/usr/bin/env python3
"""Bridge original go2_robot_sdk topics into the AEDE real-robot interface."""

import math
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros import StaticTransformBroadcaster


class Go2AedeBridge(Node):
    """Thin topic/TF bridge between the Go2 SDK and AEDE."""

    def __init__(self):
        super().__init__('go2_aede_bridge')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('sensor_frame', 'sensor')
        self.declare_parameter('sensor_x', 0.0)
        self.declare_parameter('sensor_y', 0.0)
        self.declare_parameter('sensor_z', 0.0)
        self.declare_parameter('sensor_roll', 0.0)
        self.declare_parameter('sensor_pitch', 0.0)
        self.declare_parameter('sensor_yaw', 0.0)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_lateral_speed', 0.3)
        self.declare_parameter('max_yaw_rate', 0.8)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
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
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_lateral_speed = float(self.get_parameter('max_lateral_speed').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)

        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tfs()

        self.state_est_pub = self.create_publisher(
            Odometry, '/state_estimation', self.reliable_qos)
        self.registered_scan_pub = self.create_publisher(
            PointCloud2, '/registered_scan', self.reliable_qos)
        self.cmd_vel_out_pub = self.create_publisher(
            Twist, '/cmd_vel_out', self.reliable_qos)

        self.create_subscription(
            Odometry, '/odom', self._on_odom, self.reliable_qos)
        self.create_subscription(
            PointCloud2, '/point_cloud2', self._on_pointcloud, self.best_effort_qos)
        self.create_subscription(
            TwistStamped, '/cmd_vel', self._on_cmd_vel, self.reliable_qos)

        self.get_logger().info('Go2-AEDE bridge started')

    def _publish_static_tfs(self) -> None:
        now = self.get_clock().now().to_msg()

        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = now
        map_to_odom.header.frame_id = self.map_frame
        map_to_odom.child_frame_id = self.odom_frame
        map_to_odom.transform.rotation.w = 1.0

        base_to_sensor = TransformStamped()
        base_to_sensor.header.stamp = now
        base_to_sensor.header.frame_id = self.base_frame
        base_to_sensor.child_frame_id = self.sensor_frame
        base_to_sensor.transform.translation.x = self.sensor_offset[0]
        base_to_sensor.transform.translation.y = self.sensor_offset[1]
        base_to_sensor.transform.translation.z = self.sensor_offset[2]
        base_to_sensor.transform.rotation.x = self.sensor_rotation[0]
        base_to_sensor.transform.rotation.y = self.sensor_rotation[1]
        base_to_sensor.transform.rotation.z = self.sensor_rotation[2]
        base_to_sensor.transform.rotation.w = self.sensor_rotation[3]

        self.static_broadcaster.sendTransform([map_to_odom, base_to_sensor])

    def _on_odom(self, msg: Odometry) -> None:
        """Publish AEDE state_estimation as the sensor pose in map frame."""
        base_position = msg.pose.pose.position
        base_orientation = msg.pose.pose.orientation
        base_quaternion = (
            base_orientation.x,
            base_orientation.y,
            base_orientation.z,
            base_orientation.w,
        )
        base_quaternion = self._normalize_quaternion(base_quaternion)
        rotated_offset = self._rotate_vector(base_quaternion, self.sensor_offset)
        sensor_quaternion = self._normalize_quaternion(
            self._quaternion_multiply(base_quaternion, self.sensor_rotation)
        )

        state = Odometry()
        state.header.stamp = msg.header.stamp
        state.header.frame_id = self.map_frame
        state.child_frame_id = self.sensor_frame
        state.pose = deepcopy(msg.pose)
        state.twist = deepcopy(msg.twist)
        state.pose.pose.position.x = base_position.x + rotated_offset[0]
        state.pose.pose.position.y = base_position.y + rotated_offset[1]
        state.pose.pose.position.z = base_position.z + rotated_offset[2]
        state.pose.pose.orientation.x = sensor_quaternion[0]
        state.pose.pose.orientation.y = sensor_quaternion[1]
        state.pose.pose.orientation.z = sensor_quaternion[2]
        state.pose.pose.orientation.w = sensor_quaternion[3]

        self.state_est_pub.publish(state)

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        """Republish Go2 global point cloud as AEDE registered_scan."""
        msg.header.frame_id = self.map_frame
        self.registered_scan_pub.publish(msg)

    def _on_cmd_vel(self, msg: TwistStamped) -> None:
        """Clamp AEDE TwistStamped and forward it to the Go2 SDK."""
        twist = Twist()
        twist.linear.x = self._clamp(
            msg.twist.linear.x, -self.max_linear_speed, self.max_linear_speed)
        twist.linear.y = self._clamp(
            msg.twist.linear.y, -self.max_lateral_speed, self.max_lateral_speed)
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self._clamp(
            msg.twist.angular.z, -self.max_yaw_rate, self.max_yaw_rate)
        self.cmd_vel_out_pub.publish(twist)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, float(value)))

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
        x, y, z, w = quaternion
        return (-x, -y, -z, w)

    @classmethod
    def _rotate_vector(cls, quaternion, vector):
        vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
        rotated = cls._quaternion_multiply(
            cls._quaternion_multiply(quaternion, vector_quaternion),
            cls._quaternion_conjugate(quaternion),
        )
        return rotated[:3]

    @staticmethod
    def _normalize_quaternion(quaternion):
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm == 0.0:
            return (0.0, 0.0, 0.0, 1.0)
        return tuple(value / norm for value in quaternion)


def main():
    rclpy.init()
    node = Go2AedeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
