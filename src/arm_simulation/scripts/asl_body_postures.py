#!/usr/bin/env python3
"""
ASL body posture data library.
Stateless module — no ROS dependencies.
Defines body joint targets for each sign command.

Body joints managed (10 total):
    left_shoulder_pitch, right_shoulder_pitch
    left_shoulder_roll,  right_shoulder_roll
    left_elbow_pitch,    right_elbow_pitch
    left_wrist_pitch,    right_wrist_pitch
    neck_pitch,          neck_yaw

Each posture is a dict mapping joint name -> target angle (radians).
Missing keys default to 0.0 (neutral).

Each sign entry in SIGN_POSTURES is a list of frames:
    {'joints': {joint: angle, ...}, 'duration': float (seconds)}
"""

# All 10 body joints at rest
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

# Signing-ready position: both arms raised in front of the body
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

# Helper: frame constructor
def _frame(joints, duration):
    return {'joints': joints, 'duration': duration}


# Wrist oscillation base for ASL "10" (thumb wave)
def _wrist_frame(angle, duration=0.5):
    return _frame({
        **SIGNING_POSE,
        'right_wrist_pitch': angle,
    }, duration)


SIGN_POSTURES = {
    # ── hello ──────────────────────────────────────────────────────────────
    # Right hand rises to forehead level, sweeps outward, returns to neutral
    'hello': [
        _frame({                             # raise to forehead
            'right_shoulder_pitch': 0.85,
            'right_shoulder_roll':  0.30,
            'right_elbow_pitch':    1.40,
            'right_wrist_pitch':    0.0,
            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   0.0,
            'left_elbow_pitch':     0.0,
            'left_wrist_pitch':     0.0,
            'neck_pitch':           0.15,
            'neck_yaw':             -0.15,  # slight turn toward signing hand
        }, 1.0),
        _frame({                             # sweep outward
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
        _frame(NEUTRAL_POSE, 0.8),           # return to neutral
    ],

    # ── numbers 1-9: one signing-ready frame each ─────────────────────────
    '1': [_frame(SIGNING_POSE, 1.5)],
    '2': [_frame(SIGNING_POSE, 1.5)],
    '3': [_frame(SIGNING_POSE, 1.5)],
    '4': [_frame(SIGNING_POSE, 1.5)],
    '5': [_frame(SIGNING_POSE, 1.5)],
    '6': [_frame(SIGNING_POSE, 1.5)],
    '7': [_frame(SIGNING_POSE, 1.5)],
    '8': [_frame(SIGNING_POSE, 1.5)],
    '9': [_frame(SIGNING_POSE, 1.5)],

    # ── 10: wrist oscillation (ASL "10" = thumb shake) ────────────────────
    '10': [
        _wrist_frame( 0.20),
        _wrist_frame(-0.20),
        _wrist_frame( 0.20),
        _wrist_frame(-0.20),
    ],
}

# Hand shape override table keyed by sign command.
# Values: {'right': gesture_name, 'left': gesture_name}
# gesture_name must be a key in asl_mapper.ASLMapper.asl_alphabet /
# asl_numbers / asl_words, or 'neutral'.
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
    '10':    {'right': 'y',     'left': 'neutral'},  # y = thumb+pinky
}
