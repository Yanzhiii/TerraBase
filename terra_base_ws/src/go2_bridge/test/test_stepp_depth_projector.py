import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, PointField

from go2_bridge.stepp_depth_projector import (
    _non_optical_frame_id,
    _stamp_to_seconds,
    _voxel_filter_latest_cost,
    decode_depth_image,
    make_pointcloud2,
    resize_nearest,
)


def test_decode_16uc1_depth_image_to_meters():
    msg = Image()
    msg.height = 2
    msg.width = 3
    msg.encoding = '16UC1'
    msg.is_bigendian = False
    msg.step = 6
    raw = np.array([[1000, 2000, 0], [1500, 2500, 500]], dtype=np.uint16)
    msg.data = raw.tobytes()

    depth = decode_depth_image(msg)

    assert depth.dtype == np.float32
    np.testing.assert_allclose(
        depth,
        np.array([[1.0, 2.0, 0.0], [1.5, 2.5, 0.5]], dtype=np.float32),
    )


def test_resize_nearest_preserves_corner_mapping():
    matrix = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)

    resized = resize_nearest(matrix, 4, 4)

    assert resized.shape == (4, 4)
    np.testing.assert_array_equal(
        resized,
        np.array([
            [1.0, 1.0, 2.0, 2.0],
            [1.0, 1.0, 2.0, 2.0],
            [3.0, 3.0, 4.0, 4.0],
            [3.0, 3.0, 4.0, 4.0],
        ], dtype=np.float32),
    )


def test_make_pointcloud2_uses_aede_cost_fields():
    points = np.array([[1.0, 2.0, 3.0, 0.2]], dtype=np.float32)
    stamp = Time(sec=12, nanosec=34)

    msg = make_pointcloud2(points, stamp, 'map')

    assert msg.header.frame_id == 'map'
    assert msg.header.stamp.sec == 12
    assert msg.width == 1
    assert msg.point_step == 16
    assert [field.name for field in msg.fields] == ['x', 'y', 'z', 'intensity']
    assert all(field.datatype == PointField.FLOAT32 for field in msg.fields)
    np.testing.assert_allclose(
        np.frombuffer(msg.data, dtype=np.float32),
        np.array([1.0, 2.0, 3.0, 0.2], dtype=np.float32),
    )


def test_non_optical_frame_id_strips_ros_optical_suffix():
    assert _non_optical_frame_id('zed_left_camera_frame_optical') == 'zed_left_camera_frame'
    assert _non_optical_frame_id('camera') == 'camera'


def test_stamp_to_seconds_uses_nanosecond_fraction():
    assert _stamp_to_seconds(Time(sec=3, nanosec=250000000)) == 3.25


def test_voxel_filter_latest_cost_prefers_latest_observation():
    points = np.array([
        [0.01, 0.01, 0.01, 0.20, 1.0],
        [0.02, 0.02, 0.02, 0.05, 2.0],
        [1.00, 0.00, 0.00, 0.10, 1.5],
    ], dtype=np.float64)

    filtered = _voxel_filter_latest_cost(points, 0.10)

    assert filtered.shape == (2, 5)
    latest_same_voxel = filtered[np.argmin(filtered[:, 0])]
    np.testing.assert_allclose(latest_same_voxel[3:], np.array([0.05, 2.0]))
