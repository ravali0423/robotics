#!/usr/bin/env python3
"""
ASL body posture data library.
Stateless module — no ROS dependencies.
Defines body joint targets for each sign command.
"""

# ─────────────────────────────────────────────────────────────
# Neutral Pose
# ─────────────────────────────────────────────────────────────
NEUTRAL_POSE = {
    'left_shoulder_pitch':  0.0,
    'left_shoulder_roll':   0.0,
    'left_elbow_pitch':     0.0,
    'left_wrist_pitch':     0.0,
    'right_shoulder_pitch': 0.0,
    'right_shoulder_roll':  0.0,
    'right_elbow_pitch':    0.0,
    'right_wrist_pitch':    0.0,
    'neck_pitch':           0.0,
    'neck_yaw':             0.0,
}

# ─────────────────────────────────────────────────────────────
# Default Signing Pose (both arms)
# ─────────────────────────────────────────────────────────────
SIGNING_POSE = {
    'left_shoulder_pitch':  0.25,
    'left_shoulder_roll':   0.05,
    'left_elbow_pitch':     0.8,
    'left_wrist_pitch':     0.0,
    'right_shoulder_pitch': 0.30,
    'right_shoulder_roll':  0.10,
    'right_elbow_pitch':    1.0,
    'right_wrist_pitch':    0.0,
    'neck_pitch':           0.1,
    'neck_yaw':             0.0,
}

# ─────────────────────────────────────────────────────────────
# ASL "1" pose — right arm only
#
# Axes confirmed from TF image:
#   X (red)   = left/right (outward from shoulders)
#   Y (green) = toward camera (forward)
#   Z (blue)  = up
#
# shoulder_roll  (X axis): positive → arm swings FORWARD toward camera
# shoulder_pitch (Y axis): positive → arm swings outward/left (lateral)
# elbow_pitch    (Y axis): positive → forearm bends (same plane as shoulder_pitch)
# wrist_pitch    (Y axis): positive → wrist tilts (adjust palm direction)
# ─────────────────────────────────────────────────────────────
RIGHT_HAND_1_POSE = {
    # LEFT arm — stays neutral
    'left_shoulder_pitch': 0.0,
    'left_shoulder_roll':  0.0,
    'left_elbow_pitch':    0.0,
    'left_wrist_pitch':    0.0,

    # RIGHT arm — raised forward, palm toward camera
    'right_shoulder_roll':  0.8,   # brings arm FORWARD toward camera (X axis, positive = toward Y)
    'right_shoulder_pitch': 0.2,   # slight inward to center arm in front of body
    'right_elbow_pitch':    1.0,   # bend elbow so forearm angles upward
    'right_wrist_pitch':    0.5,   # tilt wrist so palm faces camera

    'neck_pitch': 0.1,
    'neck_yaw':   0.0,
}

# ─────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────
def _frame(joints, duration):
    return {'joints': joints, 'duration': duration}


def _wrist_frame(angle, duration=0.5):
    return _frame({
        **SIGNING_POSE,
        'right_wrist_pitch': angle,
    }, duration)


# ─────────────────────────────────────────────────────────────
# Sign Definitions
# ─────────────────────────────────────────────────────────────
SIGN_POSTURES = {

    # ── hello ────────────────────────────────────────────────
    'hello': [
        _frame({
            'right_shoulder_pitch': 0.85,
            'right_shoulder_roll':  0.30,
            'right_elbow_pitch':    1.40,
            'right_wrist_pitch':    0.0,
            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   0.0,
            'left_elbow_pitch':     0.0,
            'left_wrist_pitch':     0.0,
            'neck_pitch':           0.15,
            'neck_yaw':            -0.15,
        }, 1.0),

        _frame({
            'right_shoulder_pitch': 0.45,
            'right_shoulder_roll':  0.65,
            'right_elbow_pitch':    1.10,
            'right_wrist_pitch':    0.0,
            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   0.0,
            'left_elbow_pitch':     0.0,
            'left_wrist_pitch':     0.0,
            'neck_pitch':           0.10,
            'neck_yaw':             0.0,
        }, 1.0),

        _frame(NEUTRAL_POSE, 0.8),
    ],

    # ── numbers ─────────────────────────────────────────────
    '1': [_frame(RIGHT_HAND_1_POSE, 1.5)],

    '2': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    '3': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    '4': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    '5': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    '6': [_frame(SIGNING_POSE, 1.5)],
    '7': [_frame(SIGNING_POSE, 1.5)],
    '8': [_frame(SIGNING_POSE, 1.5)],
    '9': [_frame(SIGNING_POSE, 1.5)],

    # ── 10 ─────────────────────────────────────────────────
    '10': [
        _wrist_frame( 0.20),
        _wrist_frame(-0.20),
        _wrist_frame( 0.20),
        _wrist_frame(-0.20),
    ],
}

# ─────────────────────────────────────────────────────────────
# Hand Shape Overrides
# ─────────────────────────────────────────────────────────────
HAND_SHAPE_OVERRIDES = {
    'hello': {'right': 'hello', 'left': 'neutral'},
    '1':     {'right': '1',     'left': 'neutral'},
    '2':     {'right': '2',     'left': 'neutral'},
    '3':     {'right': '3',     'left': 'neutral'},
    '4':     {'right': '4',     'left': 'neutral'},
    '5':     {'right': '5',     'left': 'neutral'},
    '6':     {'right': '6',     'left': 'neutral'},
    '7':     {'right': '7',     'left': 'neutral'},
    '8':     {'right': '8',     'left': 'neutral'},
    '9':     {'right': '9',     'left': 'neutral'},
    '10':    {'right': 'y',     'left': 'neutral'},
}