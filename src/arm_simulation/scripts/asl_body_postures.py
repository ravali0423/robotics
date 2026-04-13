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
# ✅ FINAL FIX: Right-hand-only ASL "1" pose (correct orientation)
# ─────────────────────────────────────────────────────────────
RIGHT_HAND_1_POSE = {
    # LEFT arm → fully neutral (no movement at all)
    'left_shoulder_pitch':  0.0,
    'left_shoulder_roll':   0.0,
    'left_elbow_pitch':     0.0,
    'left_wrist_pitch':     0.0,

    # RIGHT arm → rotated inward + forward
    'right_shoulder_pitch': 0.60,   # more forward lift
    'right_shoulder_roll': -0.60,   # strong inward rotation (KEY FIX)
    'right_elbow_pitch':    1.35,   # bend forward correctly

    # Wrist → face camera
    'right_wrist_pitch':   -0.90,

    # Head (optional realism)
    'neck_pitch':           0.1,
    'neck_yaw':             0.0,
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

    '2': [_frame(SIGNING_POSE, 1.5)],
    '3': [_frame(SIGNING_POSE, 1.5)],
    '4': [_frame(SIGNING_POSE, 1.5)],
    '5': [_frame(SIGNING_POSE, 1.5)],
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