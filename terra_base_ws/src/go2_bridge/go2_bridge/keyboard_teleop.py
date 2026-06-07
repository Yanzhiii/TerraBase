#!/usr/bin/env python3
"""
Keyboard teleoperation node for Go2 robot.

Publishes Twist on /cmd_vel_out for movement.
Sends stand_up / stand_down commands via Go2's WebRTC API.

Controls:
    W / S     : forward / backward
    A / D     : left / right
    Q / E     : rotate left / right
    Space     : stand up
    X         : stand down
    1 / 2 / 3 : speed level (slow / medium / fast)
    ESC / Ctrl+C : quit

Usage:
    ros2 run go2_bridge keyboard_teleop
    # or with custom speed:
    ros2 run go2_bridge keyboard_teleop --ros-args -p max_speed:=1.5
"""

import os
import sys
import select
import termios
import tty
import threading
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Speed presets
SPEED_PRESETS = {
    '1': {'linear': 0.3, 'angular': 0.5},   # slow
    '2': {'linear': 0.7, 'angular': 1.0},   # medium
    '3': {'linear': 1.2, 'angular': 1.5},   # fast
}


class KeyboardTeleop(Node):
    """Keyboard teleoperation node for Go2 robot."""

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('max_speed', 0.7)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('publish_rate', 20.0)

        self.max_speed = self.get_parameter('max_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.publish_rate = self.get_parameter('publish_rate').value

        qos = QoSProfile(depth=10)

        # Publisher for movement commands (consumed by go2_driver)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_out', qos)

        # Publisher for stand/other commands (consumed by go2_driver_node
        # if extended, or directly by a service - kept for future use)
        self.ctrl_pub = self.create_publisher(String, '/go2_control', qos)

        # State
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.speed_level = '2'  # default: medium
        self.running = True

        # Start keyboard reader thread
        self.key_thread = threading.Thread(target=self._keyboard_loop,
                                          daemon=True)
        self.key_thread.start()

        # Timer for publishing
        period = 1.0 / self.publish_rate
        self.pub_timer = self.create_timer(period, self._publish_cmd)

        self._print_help()

    # ---- Command publishing ----

    def _publish_cmd(self):
        """Publish current velocity command."""
        twist = Twist()
        twist.linear.x = self.linear_x
        twist.linear.y = self.linear_y
        twist.angular.z = self.angular_z
        self.cmd_pub.publish(twist)

    def _send_control(self, command: str, param: str = ''):
        """Send a control command string."""
        msg = String()
        msg.data = json.dumps({'cmd': command, 'param': param})
        self.ctrl_pub.publish(msg)

    # ---- Keyboard handling ----

    def _keyboard_loop(self):
        """Non-blocking keyboard input loop."""
        old_settings = None
        try:
            fd = sys.stdin.fileno()
            if not os.isatty(fd):
                self.get_logger().warn(
                    'stdin is not a TTY — keyboard input disabled. '
                    'Run in a real terminal for keyboard control.')
                # Keep publishing zeros, stay alive
                while self.running and rclpy.ok():
                    import time
                    time.sleep(0.5)
                return

            # Save terminal settings, switch to raw mode
            old_settings = termios.tcgetattr(fd)
            tty.setraw(fd)

            while self.running and rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    self._handle_key(key)

        except Exception as e:
            self.get_logger().error(f'Keyboard error: {e}')
        finally:
            if old_settings:
                try:
                    termios.tcsetattr(sys.stdin.fileno(),
                                     termios.TCSADRAIN, old_settings)
                except Exception:
                    pass

    def _handle_key(self, key: str):
        """Process a single keypress."""
        # Speed level
        if key in SPEED_PRESETS:
            self.speed_level = key
            self.max_speed = SPEED_PRESETS[key]['linear']
            self.max_yaw_rate = SPEED_PRESETS[key]['angular']
            level_names = {'1': 'SLOW', '2': 'MEDIUM', '3': 'FAST'}
            self.get_logger().info(
                f'Speed: {level_names[key]} '
                f'(v={self.max_speed}, ω={self.max_yaw_rate})')

        # Movement
        elif key == 'w':
            self.linear_x = self.max_speed
        elif key == 's':
            self.linear_x = -self.max_speed
        elif key == 'a':
            self.linear_y = self.max_speed
        elif key == 'd':
            self.linear_y = -self.max_speed
        elif key == 'q':
            self.angular_z = self.max_yaw_rate
        elif key == 'e':
            self.angular_z = -self.max_yaw_rate

        # Stop (release movement keys)
        elif key == ' ':
            # Space: stop + stand up
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0
            self._send_control('stand_up')
            self.get_logger().info('Command: STAND UP')

        elif key == 'x':
            # X: stop + stand down
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0
            self._send_control('stand_down')
            self.get_logger().info('Command: STAND DOWN')

        # Stop all movement (release)
        elif key in ('\x03', '\x1b'):  # Ctrl+C or ESC
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0
            self.running = False
            self.get_logger().info('Quitting...')

        else:
            # Any other key: stop
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0

    def _print_help(self):
        """Print control instructions."""
        msg = """
╔══════════════════════════════════════════════╗
║       Go2 Keyboard Teleop Controls          ║
╠══════════════════════════════════════════════╣
║  W/S    : forward / backward                ║
║  A/D    : strafe left / right               ║
║  Q/E    : rotate left / right               ║
║  Space  : stand up (+ stop)                 ║
║  X      : stand down (+ stop)               ║
║  1/2/3  : speed SLOW/MEDIUM/FAST            ║
║  Other  : stop movement                     ║
║  ESC    : quit                              ║
╠══════════════════════════════════════════════╣
║  Speed 1 (SLOW) :  v=0.3  ω=0.5            ║
║  Speed 2 (MED)  :  v=0.7  ω=1.0            ║
║  Speed 3 (FAST) :  v=1.2  ω=1.5            ║
╚══════════════════════════════════════════════╝
"""
        self.get_logger().info(msg)

    def destroy_node(self):
        self.running = False
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        # Send zero command on exit
        twist = Twist()
        self.cmd_pub.publish(twist)
        super().destroy_node()


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
