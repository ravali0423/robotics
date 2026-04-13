#!/usr/bin/env python3
"""
body_controller.py — ROS 2 node that manages the 10 humanoid body joints.

Publishes /joint_states at 10 Hz with smooth interpolation toward target angles.
Subscribes to /body_pose_command (std_msgs/String) which accepts:
  - A known pose name: "neutral", "signing_ready"
  - A JSON object string:  '{"right_shoulder_pitch": 0.5, "neck_yaw": -0.2}'

Publishes /body_status (std_msgs/String) with "ready" | "moving" | <pose_name>.

Pattern mirrors enhanced_hand_controller.py.
"""

import json
import sys
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# Allow importing asl_body_postures from the same installed lib directory
try:
    from ament_index_python.packages import get_package_prefix
    _pkg_lib = os.path.join(get_package_prefix('arm_simulation'), 'lib', 'arm_simulation')
    if _pkg_lib not in sys.path:
        sys.path.insert(0, _pkg_lib)
except Exception:
    pass

from asl_body_postures import NEUTRAL_POSE, SIGNING_POSE

BODY_JOINTS = [
    'left_shoulder_pitch',
    'left_shoulder_roll',
    'left_elbow_pitch',
    'left_wrist_pitch',
    'right_shoulder_pitch',
    'right_shoulder_roll',
    'right_elbow_pitch',
    'right_wrist_pitch',
    'neck_pitch',
    'neck_yaw',
]

NAMED_POSES = {
    'neutral':       NEUTRAL_POSE,
    'signing_ready': SIGNING_POSE,
}

PUBLISH_RATE_HZ   = 10.0
TRANSITION_SPEED  = 0.3   # fraction of remaining gap closed each tick


class BodyController(Node):
    def __init__(self):
        super().__init__('body_controller')

        self._current  = {j: NEUTRAL_POSE.get(j, 0.0) for j in BODY_JOINTS}
        self._target   = {j: NEUTRAL_POSE.get(j, 0.0) for j in BODY_JOINTS}
        self._status   = 'ready'

        self._joint_pub  = self.create_publisher(JointState, '/joint_states', 10)
        self._status_pub = self.create_publisher(String, '/body_status', 10)

        self.create_subscription(
            String, '/body_pose_command', self._on_pose_command, 10)

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

        self.get_logger().info('body_controller ready')

    # ------------------------------------------------------------------ #
    # Subscription callback
    # ------------------------------------------------------------------ #
    def _on_pose_command(self, msg: String):
        data = msg.data.strip()

        if data in NAMED_POSES:
            self._apply_pose(NAMED_POSES[data])
            self._status = data
            return

        # Try JSON dict
        try:
            overrides = json.loads(data)
            if isinstance(overrides, dict):
                new_target = dict(self._target)
                for joint, angle in overrides.items():
                    if joint in BODY_JOINTS:
                        new_target[joint] = float(angle)
                    else:
                        self.get_logger().warning(f'Unknown body joint: {joint}')
                self._target = new_target
                self._status = 'moving'
                return
        except (json.JSONDecodeError, TypeError, ValueError):
            pass

        self.get_logger().warning(f'Unrecognised body_pose_command: "{data}"')

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    def _apply_pose(self, pose: dict):
        for joint in BODY_JOINTS:
            self._target[joint] = pose.get(joint, 0.0)

    def _tick(self):
        moving = False
        for joint in BODY_JOINTS:
            gap = self._target[joint] - self._current[joint]
            if abs(gap) > 1e-4:
                self._current[joint] += gap * TRANSITION_SPEED
                moving = True

        if not moving and self._status == 'moving':
            self._status = 'ready'

        self._publish_joints()
        self._publish_status()

    def _publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = BODY_JOINTS
        msg.position = [self._current[j] for j in BODY_JOINTS]
        self._joint_pub.publish(msg)

    def _publish_status(self):
        msg = String()
        msg.data = self._status
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BodyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
