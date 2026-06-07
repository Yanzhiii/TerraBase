#!/usr/bin/env python3
"""Project STEPP image costs and ZED depth into an AEDE terrain cost cloud."""

import math
import sys
from typing import Optional, Tuple

import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

from stepp_ros2_humble.msg import Float32Stamped


_OPTICAL_FRAME_SUFFIX = '_optical'


def decode_depth_image(msg: Image) -> np.ndarray:
    """Return a float32 depth image in meters."""
    encoding = msg.encoding.upper()
    if encoding == '32FC1':
        depth = _image_to_array(msg, np.float32)
    elif encoding == '16UC1':
        depth = _image_to_array(msg, np.uint16).astype(np.float32) * 0.001
    else:
        raise ValueError(f'unsupported depth encoding: {msg.encoding}')
    return depth.astype(np.float32, copy=False)


def cost_matrix_from_msg(msg: Float32Stamped) -> np.ndarray:
    """Convert STEPP Float32Stamped payload to a 2D float32 cost matrix."""
    if len(msg.data.layout.dim) < 2:
        raise ValueError('STEPP cost message has fewer than two layout dimensions')

    rows = int(msg.data.layout.dim[0].size)
    cols = int(msg.data.layout.dim[1].size)
    if rows <= 0 or cols <= 0:
        raise ValueError(f'invalid STEPP cost layout: {rows}x{cols}')

    cost = np.asarray(msg.data.data, dtype=np.float32)
    expected = rows * cols
    if cost.size < expected:
        raise ValueError(f'STEPP cost payload has {cost.size} values, expected {expected}')
    return cost[:expected].reshape((rows, cols))


def resize_nearest(matrix: np.ndarray, height: int, width: int) -> np.ndarray:
    """Resize a 2D matrix with nearest-neighbor sampling."""
    if matrix.shape == (height, width):
        return matrix
    if height <= 0 or width <= 0:
        raise ValueError(f'invalid target size: {height}x{width}')

    src_h, src_w = matrix.shape
    row_idx = np.floor(np.arange(height, dtype=np.float64) * src_h / height).astype(np.int64)
    col_idx = np.floor(np.arange(width, dtype=np.float64) * src_w / width).astype(np.int64)
    row_idx = np.clip(row_idx, 0, src_h - 1)
    col_idx = np.clip(col_idx, 0, src_w - 1)
    return matrix[row_idx[:, None], col_idx[None, :]]


def make_pointcloud2(points: np.ndarray, stamp, frame_id: str) -> PointCloud2:
    """Build a PointCloud2 with x, y, z, intensity float32 fields."""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = np.asarray(points, dtype=np.float32).tobytes()
    return msg


def _image_to_array(msg: Image, dtype) -> np.ndarray:
    dtype = np.dtype(dtype)
    row_items = int(msg.step // dtype.itemsize)
    data = np.frombuffer(msg.data, dtype=dtype)
    if msg.is_bigendian != (sys.byteorder == 'big'):
        data = data.byteswap().view(dtype.newbyteorder())
    array = data.reshape((msg.height, row_items))[:, :msg.width]
    return array.copy()


def _quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.eye(3, dtype=np.float64)

    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def _euler_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
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


def _matrix_to_quaternion(matrix: np.ndarray) -> Tuple[float, float, float, float]:
    trace = float(matrix[0, 0] + matrix[1, 1] + matrix[2, 2])
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return (
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        )

    if matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        return (
            0.25 * scale,
            (matrix[0, 1] + matrix[1, 0]) / scale,
            (matrix[0, 2] + matrix[2, 0]) / scale,
            (matrix[2, 1] - matrix[1, 2]) / scale,
        )

    if matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        return (
            (matrix[0, 1] + matrix[1, 0]) / scale,
            0.25 * scale,
            (matrix[1, 2] + matrix[2, 1]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
        )

    scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
    return (
        (matrix[0, 2] + matrix[2, 0]) / scale,
        (matrix[1, 2] + matrix[2, 1]) / scale,
        0.25 * scale,
        (matrix[1, 0] - matrix[0, 1]) / scale,
    )


def _non_optical_frame_id(frame_id: str) -> str:
    if frame_id.endswith(_OPTICAL_FRAME_SUFFIX):
        return frame_id[:-len(_OPTICAL_FRAME_SUFFIX)]
    return frame_id


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _voxel_filter_max_cost(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0.0 or points.shape[0] < 2:
        return points

    voxel_idx = np.floor(points[:, :3] / voxel_size).astype(np.int64)
    _, inverse = np.unique(voxel_idx, axis=0, return_inverse=True)
    voxel_count = int(inverse.max()) + 1
    if voxel_count == points.shape[0]:
        return points

    sums = np.zeros((voxel_count, 3), dtype=np.float64)
    counts = np.bincount(inverse, minlength=voxel_count).astype(np.float64)
    np.add.at(sums, inverse, points[:, :3])

    max_cost = np.full(voxel_count, -np.inf, dtype=np.float64)
    np.maximum.at(max_cost, inverse, points[:, 3])

    filtered = np.empty((voxel_count, 4), dtype=np.float32)
    filtered[:, :3] = (sums / counts[:, None]).astype(np.float32)
    filtered[:, 3] = max_cost.astype(np.float32)
    return filtered


def _voxel_filter_latest_cost(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0.0 or points.shape[0] < 2:
        return points

    voxel_idx = np.floor(points[:, :3] / voxel_size).astype(np.int64)
    _, inverse = np.unique(voxel_idx, axis=0, return_inverse=True)
    voxel_count = int(inverse.max()) + 1
    if voxel_count == points.shape[0]:
        return points

    latest_time = np.full(voxel_count, -np.inf, dtype=np.float64)
    np.maximum.at(latest_time, inverse, points[:, 4])

    latest_mask = points[:, 4] == latest_time[inverse]
    latest_points = points[latest_mask]
    latest_inverse = inverse[latest_mask]

    sums = np.zeros((voxel_count, 3), dtype=np.float64)
    counts = np.bincount(latest_inverse, minlength=voxel_count).astype(np.float64)
    np.add.at(sums, latest_inverse, latest_points[:, :3])

    max_cost = np.full(voxel_count, -np.inf, dtype=np.float64)
    np.maximum.at(max_cost, latest_inverse, latest_points[:, 3])

    filtered = np.empty((voxel_count, 5), dtype=np.float64)
    filtered[:, :3] = sums / counts[:, None]
    filtered[:, 3] = max_cost
    filtered[:, 4] = latest_time
    return filtered


class SteppDepthProjector(Node):
    """Convert STEPP image-space cost and depth into map-frame AEDE terrain cost."""

    def __init__(self):
        super().__init__('stepp_depth_projector')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('sensor_frame', 'sensor')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('tf_camera_frame', '')
        self.declare_parameter('sensor_to_camera_x', 0.0)
        self.declare_parameter('sensor_to_camera_y', 0.0)
        self.declare_parameter('sensor_to_camera_z', 0.0)
        self.declare_parameter('sensor_to_camera_roll', 0.0)
        self.declare_parameter('sensor_to_camera_pitch', 0.0)
        self.declare_parameter('sensor_to_camera_yaw', 0.0)
        self.declare_parameter('min_depth', 0.2)
        self.declare_parameter('max_depth', 5.0)
        self.declare_parameter('pixel_stride', 2)
        self.declare_parameter('voxel_size', 0.10)
        self.declare_parameter('max_points', 120000)
        self.declare_parameter('map_decay_time', 3.0)
        self.declare_parameter('map_memory_radius', 6.0)
        self.declare_parameter('clearing_depth_margin', 0.25)
        self.declare_parameter('sync_slop', 1.5)
        self.declare_parameter('publish_static_tf', True)

        self.map_frame = self.get_parameter('map_frame').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tf_camera_frame = self.get_parameter('tf_camera_frame').value
        if not self.tf_camera_frame and self.camera_frame:
            self.tf_camera_frame = _non_optical_frame_id(self.camera_frame)
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.pixel_stride = max(1, int(self.get_parameter('pixel_stride').value))
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_points = max(0, int(self.get_parameter('max_points').value))
        self.map_decay_time = max(0.0, float(self.get_parameter('map_decay_time').value))
        self.map_memory_radius = max(0.0, float(self.get_parameter('map_memory_radius').value))
        self.clearing_depth_margin = max(
            0.0,
            float(self.get_parameter('clearing_depth_margin').value),
        )
        self.publish_static_tf = bool(self.get_parameter('publish_static_tf').value)

        self.sensor_to_camera_translation = np.array([
            float(self.get_parameter('sensor_to_camera_x').value),
            float(self.get_parameter('sensor_to_camera_y').value),
            float(self.get_parameter('sensor_to_camera_z').value),
        ], dtype=np.float64)
        self.sensor_to_camera_rotation = _euler_to_matrix(
            float(self.get_parameter('sensor_to_camera_roll').value),
            float(self.get_parameter('sensor_to_camera_pitch').value),
            float(self.get_parameter('sensor_to_camera_yaw').value),
        )
        self.sensor_to_camera_quaternion = _euler_to_quaternion(
            float(self.get_parameter('sensor_to_camera_roll').value),
            float(self.get_parameter('sensor_to_camera_pitch').value),
            float(self.get_parameter('sensor_to_camera_yaw').value),
        )
        optical_to_camera_frame_rotation = _euler_to_matrix(-math.pi / 2.0, 0.0, -math.pi / 2.0)
        self.sensor_to_non_optical_rotation = (
            self.sensor_to_camera_rotation @ optical_to_camera_frame_rotation.T
        )
        self.sensor_to_non_optical_quaternion = _matrix_to_quaternion(
            self.sensor_to_non_optical_rotation
        )

        self.camera_info: Optional[CameraInfo] = None
        self.fx = self.fy = self.cx = self.cy = None
        self.static_tf_sent = False
        self.map_memory_cloud = np.empty((0, 5), dtype=np.float64)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.cloud_pub = self.create_publisher(PointCloud2, '/depth_projection', reliable_qos)

        self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self._on_camera_info,
            sensor_qos,
        )

        depth_sub = message_filters.Subscriber(
            self,
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            qos_profile=sensor_qos,
        )
        odom_sub = message_filters.Subscriber(
            self,
            Odometry,
            '/state_estimation',
            qos_profile=reliable_qos,
        )
        cost_sub = message_filters.Subscriber(
            self,
            Float32Stamped,
            '/inference/results_stamped_post',
            qos_profile=reliable_qos,
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [depth_sub, odom_sub, cost_sub],
            queue_size=10,
            slop=float(self.get_parameter('sync_slop').value),
        )
        self.sync.registerCallback(self._on_synced_inputs)

        if self.camera_frame:
            self._publish_static_tf()

        self.get_logger().info(
            'STEPP depth projector started '
            f'(memory={self.map_decay_time:.2f}s, radius={self.map_memory_radius:.2f}m)'
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        fx = float(msg.k[0]) if msg.k[0] else float(msg.p[0])
        fy = float(msg.k[4]) if msg.k[4] else float(msg.p[5])
        cx = float(msg.k[2]) if msg.k[2] else float(msg.p[2])
        cy = float(msg.k[5]) if msg.k[5] else float(msg.p[6])
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning('Ignoring CameraInfo with invalid focal length')
            return

        self.camera_info = msg
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        if not self.camera_frame and msg.header.frame_id:
            self.camera_frame = msg.header.frame_id
            if not self.tf_camera_frame:
                self.tf_camera_frame = _non_optical_frame_id(self.camera_frame)
            self._publish_static_tf()
            self.get_logger().info(f'Using CameraInfo frame as camera_frame: {self.camera_frame}')

    def _publish_static_tf(self) -> None:
        if self.static_tf_sent or not self.publish_static_tf or not self.tf_camera_frame:
            return

        rotation = self._tf_rotation_quaternion()
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.sensor_frame
        transform.child_frame_id = self.tf_camera_frame
        transform.transform.translation.x = float(self.sensor_to_camera_translation[0])
        transform.transform.translation.y = float(self.sensor_to_camera_translation[1])
        transform.transform.translation.z = float(self.sensor_to_camera_translation[2])
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        self.static_broadcaster.sendTransform(transform)
        self.static_tf_sent = True
        self.get_logger().info(
            f'Published static TF {self.sensor_frame} -> {self.tf_camera_frame}'
        )

    def _tf_rotation_quaternion(self) -> Tuple[float, float, float, float]:
        if (
            self.camera_frame
            and self.camera_frame.endswith(_OPTICAL_FRAME_SUFFIX)
            and self.tf_camera_frame == _non_optical_frame_id(self.camera_frame)
        ):
            return self.sensor_to_non_optical_quaternion
        return self.sensor_to_camera_quaternion

    def _on_synced_inputs(self, depth_msg: Image, odom_msg: Odometry, cost_msg: Float32Stamped) -> None:
        if self.camera_info is None:
            self.get_logger().warning('Waiting for CameraInfo before publishing STEPP terrain cloud')
            return

        try:
            depth = decode_depth_image(depth_msg)
            cost = cost_matrix_from_msg(cost_msg)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return

        if cost.shape != depth.shape:
            cost = resize_nearest(cost, depth.shape[0], depth.shape[1])

        current_cloud = self._project(depth, cost, odom_msg)
        if current_cloud.shape[0] > 0:
            current_cloud = _voxel_filter_max_cost(current_cloud, self.voxel_size)

        cloud = self._update_map_memory(
            current_cloud,
            depth,
            odom_msg,
            _stamp_to_seconds(depth_msg.header.stamp),
        )
        if cloud.shape[0] == 0:
            return
        if self.max_points and cloud.shape[0] > self.max_points:
            keep = np.linspace(0, cloud.shape[0] - 1, self.max_points).astype(np.int64)
            cloud = cloud[keep]

        self.cloud_pub.publish(make_pointcloud2(cloud, depth_msg.header.stamp, self.map_frame))

    def _update_map_memory(
        self,
        current_cloud: np.ndarray,
        depth: np.ndarray,
        odom_msg: Odometry,
        stamp_seconds: float,
    ) -> np.ndarray:
        if self.map_decay_time <= 0.0:
            self.map_memory_cloud = np.empty((0, 5), dtype=np.float64)
            return current_cloud

        if not math.isfinite(stamp_seconds) or stamp_seconds <= 0.0:
            stamp_seconds = self.get_clock().now().nanoseconds * 1e-9

        memory = self.map_memory_cloud
        if memory.shape[0] > 0:
            age = stamp_seconds - memory[:, 4]
            keep = age <= self.map_decay_time

            if self.map_memory_radius > 0.0:
                position = odom_msg.pose.pose.position
                dx = memory[:, 0] - float(position.x)
                dy = memory[:, 1] - float(position.y)
                keep &= (dx * dx + dy * dy) <= self.map_memory_radius * self.map_memory_radius

            memory = memory[keep]
            memory = self._clear_observed_memory(memory, depth, odom_msg)

        if current_cloud.shape[0] > 0:
            current_memory = np.empty((current_cloud.shape[0], 5), dtype=np.float64)
            current_memory[:, :4] = current_cloud.astype(np.float64)
            current_memory[:, 4] = stamp_seconds
            if memory.shape[0] > 0:
                memory = np.vstack([memory, current_memory])
            else:
                memory = current_memory

        if memory.shape[0] > 0:
            memory = _voxel_filter_latest_cost(memory, self.voxel_size)
            memory = self._cap_map_memory(memory, odom_msg)

        self.map_memory_cloud = memory
        return memory[:, :4].astype(np.float32, copy=False)

    def _clear_observed_memory(
        self,
        memory: np.ndarray,
        depth: np.ndarray,
        odom_msg: Odometry,
    ) -> np.ndarray:
        if (
            memory.shape[0] == 0
            or self.clearing_depth_margin <= 0.0
            or self.fx is None
            or self.fy is None
            or self.cx is None
            or self.cy is None
        ):
            return memory

        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        map_to_sensor_rotation = _quaternion_to_matrix(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        map_to_sensor_translation = np.array(
            [position.x, position.y, position.z],
            dtype=np.float64,
        )

        sensor_points = (
            map_to_sensor_rotation.T @ (memory[:, :3] - map_to_sensor_translation).T
        ).T
        camera_points = (
            self.sensor_to_camera_rotation.T
            @ (sensor_points - self.sensor_to_camera_translation).T
        ).T

        z = camera_points[:, 2]
        candidate_idx = np.where(
            np.isfinite(z)
            & (z >= self.min_depth)
            & (z <= self.max_depth)
        )[0]
        if candidate_idx.size == 0:
            return memory

        candidate_points = camera_points[candidate_idx]
        candidate_z = z[candidate_idx]
        with np.errstate(divide='ignore', invalid='ignore'):
            u = np.rint(self.fx * candidate_points[:, 0] / candidate_z + self.cx).astype(np.int64)
            v = np.rint(self.fy * candidate_points[:, 1] / candidate_z + self.cy).astype(np.int64)

        in_image = (
            (u >= 0)
            & (u < depth.shape[1])
            & (v >= 0)
            & (v < depth.shape[0])
        )
        if not np.any(in_image):
            return memory

        visible_idx = candidate_idx[in_image]
        visible_z = candidate_z[in_image]
        observed_depth = depth[v[in_image], u[in_image]]
        valid_observation = (
            np.isfinite(observed_depth)
            & (observed_depth >= self.min_depth)
            & (observed_depth <= self.max_depth)
        )
        cleared_visible = valid_observation & (
            observed_depth > visible_z + self.clearing_depth_margin
        )
        if not np.any(cleared_visible):
            return memory

        keep = np.ones(memory.shape[0], dtype=bool)
        keep[visible_idx[cleared_visible]] = False
        return memory[keep]

    def _cap_map_memory(self, memory: np.ndarray, odom_msg: Odometry) -> np.ndarray:
        if self.max_points <= 0 or memory.shape[0] <= self.max_points:
            return memory

        position = odom_msg.pose.pose.position
        dx = memory[:, 0] - float(position.x)
        dy = memory[:, 1] - float(position.y)
        dist2 = dx * dx + dy * dy
        keep = np.argpartition(dist2, self.max_points - 1)[:self.max_points]
        return memory[keep]

    def _project(self, depth: np.ndarray, cost: np.ndarray, odom_msg: Odometry) -> np.ndarray:
        rows = np.arange(0, depth.shape[0], self.pixel_stride, dtype=np.int32)
        cols = np.arange(0, depth.shape[1], self.pixel_stride, dtype=np.int32)
        vv, uu = np.meshgrid(rows, cols, indexing='ij')

        sampled_depth = depth[vv, uu]
        sampled_cost = cost[vv, uu]
        valid = (
            np.isfinite(sampled_depth)
            & np.isfinite(sampled_cost)
            & (sampled_depth >= self.min_depth)
            & (sampled_depth <= self.max_depth)
        )
        if not np.any(valid):
            return np.empty((0, 4), dtype=np.float32)

        z = sampled_depth[valid].astype(np.float64)
        u = uu[valid].astype(np.float64)
        v = vv[valid].astype(np.float64)

        camera_points = np.column_stack([
            (u - self.cx) / self.fx * z,
            (v - self.cy) / self.fy * z,
            z,
        ])

        sensor_points = (
            self.sensor_to_camera_rotation @ camera_points.T
        ).T + self.sensor_to_camera_translation

        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        map_to_sensor_rotation = _quaternion_to_matrix(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        map_to_sensor_translation = np.array(
            [position.x, position.y, position.z],
            dtype=np.float64,
        )
        map_points = (map_to_sensor_rotation @ sensor_points.T).T + map_to_sensor_translation

        output = np.empty((map_points.shape[0], 4), dtype=np.float32)
        output[:, :3] = map_points.astype(np.float32)
        output[:, 3] = sampled_cost[valid].astype(np.float32)
        return output


def main():
    rclpy.init()
    node = SteppDepthProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
