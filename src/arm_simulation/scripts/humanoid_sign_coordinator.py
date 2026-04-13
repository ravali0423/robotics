#!/usr/bin/env python3
"""
humanoid_sign_coordinator.py — top-level sign language orchestrator.

Subscribes to /sign_command (std_msgs/String).
Supported commands: "hello", "1" – "10"

For each command it:
  1. Sends hand shapes to /right/gesture_command and /left/gesture_command
  2. Drives body posture frame-by-frame via /body_pose_command (JSON)
  3. Publishes status to /sign_status: "performing" | "complete" | "busy"

Sign execution runs in a daemon thread so ROS spin is never blocked.
An _active flag prevents overlapping signs.
"""

import json
import sys
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Allow importing sibling scripts installed in the same lib directory
try:
    from ament_index_python.packages import get_package_prefix
    _pkg_lib = os.path.join(get_package_prefix('arm_simulation'), 'lib', 'arm_simulation')
    if _pkg_lib not in sys.path:
        sys.path.insert(0, _pkg_lib)
except Exception:
    pass

from asl_body_postures import NEUTRAL_POSE, SIGN_POSTURES, HAND_SHAPE_OVERRIDES


class HumanoidSignCoordinator(Node):
    def __init__(self):
        super().__init__('humanoid_sign_coordinator')

        self._active = False
        self._lock   = threading.Lock()

        # Publishers
        self._right_hand_pub = self.create_publisher(
            String, '/right/gesture_command', 10)
        self._left_hand_pub  = self.create_publisher(
            String, '/left/gesture_command',  10)
        self._body_pub       = self.create_publisher(
            String, '/body_pose_command', 10)
        self._status_pub     = self.create_publisher(
            String, '/sign_status', 10)

        # Subscriber
        self.create_subscription(
            String, '/sign_command', self._on_sign_command, 10)

        self.get_logger().info(
            'humanoid_sign_coordinator ready — supported commands: '
            'hello, 1-10')

    # ------------------------------------------------------------------ #
    # Incoming sign command
    # ------------------------------------------------------------------ #
    def _on_sign_command(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd not in SIGN_POSTURES:
            self.get_logger().warning(
                f'Unknown sign command: "{cmd}". '
                f'Supported: {sorted(SIGN_POSTURES.keys())}')
            return

        with self._lock:
            if self._active:
                self._publish_status('busy')
                self.get_logger().info(
                    f'Sign "{cmd}" ignored — already performing a sign.')
                return
            self._active = True

        t = threading.Thread(target=self._perform_sign, args=(cmd,), daemon=True)
        t.start()

    # ------------------------------------------------------------------ #
    # Sign execution (runs in daemon thread)
    # ------------------------------------------------------------------ #
    def _perform_sign(self, cmd: str):
        try:
            self._publish_status('performing')
            self.get_logger().info(f'Performing sign: {cmd}')

            # 1. Set hand shapes before the arms start moving
            hand_shapes = HAND_SHAPE_OVERRIDES.get(cmd, {'right': 'neutral', 'left': 'neutral'})
            self._publish_hand(hand_shapes['right'], hand_shapes['left'])

            # Small pause so hand shape is applied before body motion begins
            time.sleep(0.1)

            # 2. Execute body posture frames
            frames = SIGN_POSTURES[cmd]
            for frame in frames:
                joint_dict = frame['joints']
                duration   = frame['duration']
                self._publish_body(joint_dict)
                time.sleep(duration)

            # 3. Return to neutral
            self._publish_body(NEUTRAL_POSE)
            time.sleep(0.8)

            # 4. Reset hands to neutral
            self._publish_hand('neutral', 'neutral')

            self._publish_status('complete')
            self.get_logger().info(f'Sign complete: {cmd}')

        except Exception as e:
            self.get_logger().error(f'Error during sign "{cmd}": {e}')
            self._publish_status('error')

        finally:
            with self._lock:
                self._active = False

    # ------------------------------------------------------------------ #
    # Publish helpers
    # ------------------------------------------------------------------ #
    def _publish_hand(self, right: str, left: str):
        r_msg = String(); r_msg.data = right
        l_msg = String(); l_msg.data = left
        self._right_hand_pub.publish(r_msg)
        self._left_hand_pub.publish(l_msg)

    def _publish_body(self, joints: dict):
        msg = String()
        msg.data = json.dumps(joints)
        self._body_pub.publish(msg)

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidSignCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
