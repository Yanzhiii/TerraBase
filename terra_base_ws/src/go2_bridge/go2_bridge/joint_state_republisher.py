#!/usr/bin/env python3
"""Publish a stable /joint_states stream from the upstream Go2 SDK output."""

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState


DEFAULT_JOINT_NAMES = [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
]


class JointStateRepublisher(Node):
    """Republish raw SDK joint states at a steady rate, with neutral startup pose."""

    def __init__(self):
        super().__init__('go2_joint_state_republisher')

        self.declare_parameter('input_topic', '/go2/joint_states_raw')
        self.declare_parameter('output_topic', '/joint_states')
        self.declare_parameter('publish_rate', 20.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.publisher = self.create_publisher(JointState, output_topic, qos)
        self.create_subscription(JointState, input_topic, self._on_joint_state, qos)

        self.last_joint_state = self._neutral_joint_state()
        self.create_timer(1.0 / publish_rate, self._on_timer)
        self.get_logger().info(
            f'Republishing {input_topic} to {output_topic} at {publish_rate:g} Hz')

    def _neutral_joint_state(self) -> JointState:
        msg = JointState()
        msg.name = list(DEFAULT_JOINT_NAMES)
        msg.position = [0.0] * len(DEFAULT_JOINT_NAMES)
        msg.velocity = []
        msg.effort = []
        return msg

    def _on_joint_state(self, msg: JointState) -> None:
        self.last_joint_state = deepcopy(msg)

    def _on_timer(self) -> None:
        msg = deepcopy(self.last_joint_state)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = JointStateRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
