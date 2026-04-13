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
    # LEFT ARM
    'left_shoulder_pitch':  0.0,
    'left_shoulder_roll':   1.5708,
    'left_elbow_pitch':     0,
    'left_wrist_pitch':    0,

    # RIGHT ARM
    'right_shoulder_pitch': 0.0,
    'right_shoulder_roll': 1.5708,
    'right_elbow_pitch':    0,
    'right_wrist_pitch':   0,

    # HEAD
    'neck_pitch':           0.0,
    'neck_yaw':             0.0,
}

# ─────────────────────────────────────────────────────────────
# Default Signing Pose (both arms)
# ─────────────────────────────────────────────────────────────
SIGNING_POSE = {
    'left_shoulder_pitch':  0.0,
    'left_shoulder_roll':   1.5708,
    'left_elbow_pitch':     0.0,
    'left_wrist_pitch':     0.0,
    'right_shoulder_pitch': 0.0,     
    'right_shoulder_roll':  1.5708,
    'right_elbow_pitch':    0.0,
    'right_wrist_pitch':    0.0,

    'neck_pitch': 0.1,
    'neck_yaw':   0.0,
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

    # RIGHT arm geometry:
    #   shoulder→elbow: straight down (-Z)  →  shoulder_pitch=0, shoulder_roll=0
    #   elbow→wrist:    toward camera (+Y)  →  Rx(π/2) rotates -Z arm to +Y
    #                   achieved by shoulder_roll = 1.5708 (90°)
    #   palm:           facing camera       →  wrist_pitch=0 (palm already faces +Y after roll)
    'right_shoulder_pitch': 0.0,     # upper arm straight down (no lateral swing)
    'right_shoulder_roll':  1.5708,  # 90° → forearm points toward camera (+Y)
    'right_elbow_pitch':    0.0,     # forearm stays pointing toward camera
    'right_wrist_pitch':    0.0,     # palm faces camera naturally after the 90° roll

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

        # Step 1 — bring hand UP toward head (diagonal lift)
        _frame({
            'right_shoulder_pitch': -0.5,   # 👈 pull inward toward head
            'right_shoulder_roll':  1.4,    # 👈 strong forward
            'right_elbow_pitch':    0.8,    # 👈 bend to bring hand closer
            'right_wrist_pitch':    0.0,

            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   0.0,
            'left_elbow_pitch':     0.0,
            'left_wrist_pitch':     0.0,

            'neck_pitch':           0.1,
            'neck_yaw':            -0.2,
        }, 1.2),

        # Step 2 — wave outward
        _frame({
            'right_shoulder_pitch': -0.2,
            'right_shoulder_roll':  1.2,
            'right_elbow_pitch':    0.8,
            'right_wrist_pitch':    0.0,

            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   0.0,
            'left_elbow_pitch':     0.0,
            'left_wrist_pitch':     0.0,

            'neck_pitch':           0.1,
            'neck_yaw':             0.0,
        }, 0.6),

        # Step 3 — wave back
        _frame({
            'right_shoulder_pitch': -0.6,
            'right_shoulder_roll':  1.3,
            'right_elbow_pitch':    0.8,
            'right_wrist_pitch':    0.0,

            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   0.0,
            'left_elbow_pitch':     0.0,
            'left_wrist_pitch':     0.0,

            'neck_pitch':           0.1,
            'neck_yaw':            -0.1,
        }, 0.6),

        # Step 4 — return
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
    '6':     {'right': '5',     'left': '1'},
    '7':     {'right': '5',     'left': '2'},
    '8':     {'right': '5',     'left': '3'},
    '9':     {'right': '5',     'left': '4'},
    '10':    {'right': '5',     'left': '5'},
}