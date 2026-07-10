import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from lite3_bridge.bridge_node import Lite3AedeBridge, PROFILE_DEFAULTS


def test_profile_defaults_are_explicit():
    assert PROFILE_DEFAULTS['raw_smoke']['odom_topic'] == '/leg_odom2'
    assert PROFILE_DEFAULTS['raw_smoke']['scan_topic'] == '/rslidar_points'
    assert PROFILE_DEFAULTS['faster_lio']['odom_topic'] == '/Odometry'
    assert PROFILE_DEFAULTS['faster_lio']['scan_topic'] == '/cloud_registered'


def test_clamp_limits_values():
    assert Lite3AedeBridge._clamp(2.0, -0.2, 0.2) == 0.2
    assert Lite3AedeBridge._clamp(-2.0, -0.2, 0.2) == -0.2
    assert Lite3AedeBridge._clamp(0.1, -0.2, 0.2) == 0.1


def test_twist_stamped_import_matches_aede_command_type():
    msg = TwistStamped()
    msg.twist.linear.x = 1.0
    assert msg.twist.linear.x == 1.0


def test_state_to_transform_uses_state_pose_and_child_frame():
    state = Odometry()
    state.header.frame_id = 'map'
    state.header.stamp.sec = 12
    state.header.stamp.nanosec = 34
    state.pose.pose.position.x = 1.0
    state.pose.pose.position.y = 2.0
    state.pose.pose.position.z = 3.0
    state.pose.pose.orientation.w = 1.0

    transform = Lite3AedeBridge._state_to_transform(state, 'sensor')

    assert transform.header.frame_id == 'map'
    assert transform.header.stamp.sec == 12
    assert transform.header.stamp.nanosec == 34
    assert transform.child_frame_id == 'sensor'
    assert transform.transform.translation.x == 1.0
    assert transform.transform.translation.y == 2.0
    assert transform.transform.translation.z == 3.0
    assert transform.transform.rotation.w == 1.0


def test_compose_transform_applies_parent_pose_and_child_calibration():
    parent_quaternion = Lite3AedeBridge._euler_to_quaternion(
        0.0, 0.0, math.pi / 2.0)
    child_rotation = Lite3AedeBridge._euler_to_quaternion(
        0.0, 0.0, math.pi / 2.0)

    child_position, child_quaternion = Lite3AedeBridge._compose_transform(
        (1.0, 2.0, 0.0),
        parent_quaternion,
        (1.0, 0.0, 0.0),
        child_rotation,
    )

    assert math.isclose(child_position[0], 1.0, abs_tol=1e-9)
    assert math.isclose(child_position[1], 3.0, abs_tol=1e-9)
    assert math.isclose(child_position[2], 0.0, abs_tol=1e-9)
    assert math.isclose(child_quaternion[2], 1.0, abs_tol=1e-9)
    assert math.isclose(child_quaternion[3], 0.0, abs_tol=1e-9)
