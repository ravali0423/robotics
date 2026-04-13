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


def get_two_hand_sign(left_shape: str, right_shape: str):
    """
    Return (frames, hand_overrides) for a simultaneous two-hand sign.
    Both arms hold SIGNING_POSE while each hand shows a different shape.
    """
    frames = [_frame(SIGNING_POSE, 1.5)]
    hand_overrides = {'right': right_shape, 'left': left_shape}
    return frames, hand_overrides


def resolve_two_hand_command(cmd: str):
    """
    Detect whether cmd should be executed as a simultaneous two-hand sign.

    Supported patterns:
      - Two-letter word  (e.g. "hi")   → left='h', right='i'
      - Two space-separated tokens     → left=token[0], right=token[1]
        Works for single letters, numbers, or known sign words.

    Returns (left_shape, right_shape, frames, hand_overrides)
    or None if the command does not match either pattern.
    """
    tokens = cmd.split()

    if len(tokens) == 2:
        left, right = tokens[0], tokens[1]
    elif len(tokens) == 1 and len(cmd) == 2 and cmd.isalpha():
        left, right = cmd[0], cmd[1]
    else:
        return None

    frames, hand_overrides = get_two_hand_sign(left, right)
    return left, right, frames, hand_overrides


# ─────────────────────────────────────────────────────────────
# Sign Definitions
# ─────────────────────────────────────────────────────────────
SIGN_POSTURES = {

    # ── hello ────────────────────────────────────────────────
    # ASL hello: flat-hand salute at the forehead, sweep outward
    'hello': [

        # Step 1 — raise hand to forehead (salute start)
        # shoulder_roll > π/2 lifts the arm above horizontal toward the head
        _frame({
            'right_shoulder_pitch': -0.4,   # inward toward centerline / face
            'right_shoulder_roll':  2.2,    # arm raised upward past horizontal
            'right_elbow_pitch':    0.6,    # forearm bent so hand reaches forehead
            'right_wrist_pitch':    0.0,

            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   0.0,
            'left_elbow_pitch':     0.0,
            'left_wrist_pitch':     0.0,

            'neck_pitch':           0.15,
            'neck_yaw':            -0.1,
        }, 1.0),

        # Step 2 — sweep hand outward away from forehead
        _frame({
            'right_shoulder_pitch':  0.3,   # sweep arm outward
            'right_shoulder_roll':   2.0,   # arm stays raised
            'right_elbow_pitch':     0.3,   # forearm extends as hand sweeps out
            'right_wrist_pitch':     0.0,

            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   0.0,
            'left_elbow_pitch':     0.0,
            'left_wrist_pitch':     0.0,

            'neck_pitch':           0.1,
            'neck_yaw':             0.0,
        }, 0.7),

        # Step 3 — return to neutral
        _frame(NEUTRAL_POSE, 0.8),
    ],

    # ── boy ─────────────────────────────────────────────────
    # ASL boy: grab brim of imaginary cap at forehead, release downward
    'why': [

        # Step 1 — brim-grab at forehead
        _frame({
            'right_shoulder_pitch': 0,  # arm raised toward head 
            'right_shoulder_roll':  -1.57,    # inward toward face
            'right_elbow_pitch':    1.57,   # forearm bent to reach forehead
            'right_wrist_pitch':    0,

            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   1.57,
            'left_elbow_pitch':     -1.57,
            'left_wrist_pitch':     0,

            'neck_pitch':           0.2,
            'neck_yaw':             0.2,
        }, 1.0),

        _frame(NEUTRAL_POSE, 0.8),

        _frame({
            'right_shoulder_pitch': 0,  # arm raised toward head 
            'right_shoulder_roll':  1.57,    # inward toward face
            'right_elbow_pitch':    -1.57,   # forearm bent to reach forehead
            'right_wrist_pitch':    0,

            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   -1.57,
            'left_elbow_pitch':     1.57,
            'left_wrist_pitch':     0,

            'neck_pitch':           -0.2,
            'neck_yaw':             -0.2,
        }, 1.0),

        _frame({
            'right_shoulder_pitch': 0,  # arm raised toward head 
            'right_shoulder_roll':  -1.57,    # inward toward face
            'right_elbow_pitch':    1.57,   # forearm bent to reach forehead
            'right_wrist_pitch':    0,

            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   1.57,
            'left_elbow_pitch':     -1.57,
            'left_wrist_pitch':     0,

            'neck_pitch':           0.2,
            'neck_yaw':             0.2,
        }, 1.0),

        _frame(NEUTRAL_POSE, 0.8),

        _frame({
            'right_shoulder_pitch': 0,  # arm raised toward head 
            'right_shoulder_roll':  1.57,    # inward toward face
            'right_elbow_pitch':    -1.57,   # forearm bent to reach forehead
            'right_wrist_pitch':    0,

            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   -1.57,
            'left_elbow_pitch':     1.57,
            'left_wrist_pitch':     0,

            'neck_pitch':           -0.2,
            'neck_yaw':             -0.2,
        }, 1.0),

        _frame({
            'right_shoulder_pitch': 0,  # arm raised toward head 
            'right_shoulder_roll':  -1.57,    # inward toward face
            'right_elbow_pitch':    1.57,   # forearm bent to reach forehead
            'right_wrist_pitch':    0,

            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   1.57,
            'left_elbow_pitch':     -1.57,
            'left_wrist_pitch':     0,

            'neck_pitch':           0.2,
            'neck_yaw':             0.2,
        }, 1.0),

        _frame(NEUTRAL_POSE, 0.8),

        _frame({
            'right_shoulder_pitch': 0,  # arm raised toward head 
            'right_shoulder_roll':  1.57,    # inward toward face
            'right_elbow_pitch':    -1.57,   # forearm bent to reach forehead
            'right_wrist_pitch':    0,

            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   -1.57,
            'left_elbow_pitch':     1.57,
            'left_wrist_pitch':     0,

            'neck_pitch':           -0.2,
            'neck_yaw':             -0.2,
        }, 1.0),

        _frame({
            'right_shoulder_pitch': 0,  # arm raised toward head 
            'right_shoulder_roll':  -1.57,    # inward toward face
            'right_elbow_pitch':    1.57,   # forearm bent to reach forehead
            'right_wrist_pitch':    0,

            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   1.57,
            'left_elbow_pitch':     -1.57,
            'left_wrist_pitch':     0,

            'neck_pitch':           0.2,
            'neck_yaw':             0.2,
        }, 1.0),

        _frame(NEUTRAL_POSE, 0.8),

        _frame({
            'right_shoulder_pitch': 0,  # arm raised toward head 
            'right_shoulder_roll':  1.57,    # inward toward face
            'right_elbow_pitch':    -1.57,   # forearm bent to reach forehead
            'right_wrist_pitch':    0,

            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   -1.57,
            'left_elbow_pitch':     1.57,
            'left_wrist_pitch':     0,

            'neck_pitch':           -0.2,
            'neck_yaw':             -0.2,
        }, 1.0),

        _frame(NEUTRAL_POSE, 0.8),
    ],

    # ── alphabet ────────────────────────────────────────────
    'a': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'b': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'c': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'd': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'e': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'f': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'g': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'h': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'i': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'j': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'k': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'l': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'm': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'n': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'o': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'p': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'q': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'r': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    's': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    't': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'u': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'v': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'w': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'x': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'y': [_frame(RIGHT_HAND_1_POSE, 1.5)],
    'z': [_frame(RIGHT_HAND_1_POSE, 1.5)],

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

    # ─────────────────────────────────────────────────────────────
    # Dance Moves
    # ─────────────────────────────────────────────────────────────

    # ── robot ────────────────────────────────────────────────────
    # Stiff, mechanical alternating arm snaps with head turns.
    'robot': [
        # snap right arm forward (elbow bent up), head right
        _frame({
            'right_shoulder_pitch': 0.0,
            'right_shoulder_roll':  1.5708,
            'right_elbow_pitch':   -1.5708,
            'right_wrist_pitch':    0.0,
            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   0.0,
            'left_elbow_pitch':     0.0,
            'left_wrist_pitch':     0.0,
            'neck_pitch': 0.0,
            'neck_yaw':   0.5,
        }, 0.35),
        # snap left arm forward (elbow bent up), head left
        _frame({
            'right_shoulder_pitch': 0.0,
            'right_shoulder_roll':  0.0,
            'right_elbow_pitch':    0.0,
            'right_wrist_pitch':    0.0,
            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   1.5708,
            'left_elbow_pitch':    -1.5708,
            'left_wrist_pitch':     0.0,
            'neck_pitch': 0.0,
            'neck_yaw':  -0.5,
        }, 0.35),
        # both arms forward, elbows bent — T-rex pose
        _frame({
            'right_shoulder_pitch':  0.0,
            'right_shoulder_roll':   1.5708,
            'right_elbow_pitch':    -1.5708,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    1.5708,
            'left_elbow_pitch':     -1.5708,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.0,
            'neck_yaw':   0.0,
        }, 0.35),
        # arms snap down — pause
        _frame(NEUTRAL_POSE, 0.25),
        # right arm raised high, head right
        _frame({
            'right_shoulder_pitch': -0.2,
            'right_shoulder_roll':   2.4,
            'right_elbow_pitch':     0.0,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    0.0,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': -0.1,
            'neck_yaw':   0.4,
        }, 0.35),
        # left arm raised high, head left
        _frame({
            'right_shoulder_pitch':  0.0,
            'right_shoulder_roll':   0.0,
            'right_elbow_pitch':     0.0,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':  -0.2,
            'left_shoulder_roll':    2.4,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': -0.1,
            'neck_yaw':  -0.4,
        }, 0.35),
        # both arms raised — victory stance
        _frame({
            'right_shoulder_pitch': -0.2,
            'right_shoulder_roll':   2.4,
            'right_elbow_pitch':     0.0,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':  -0.2,
            'left_shoulder_roll':    2.4,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': -0.15,
            'neck_yaw':   0.0,
        }, 0.4),
        _frame(NEUTRAL_POSE, 0.5),
    ],

    # ── celebrate ────────────────────────────────────────────────
    # Both arms shoot up and wave excitedly.
    'celebrate': [
        # arms up, wrists tilted out
        _frame({
            'right_shoulder_pitch': -0.3,
            'right_shoulder_roll':   2.7,
            'right_elbow_pitch':     0.4,
            'right_wrist_pitch':     0.4,
            'left_shoulder_pitch':  -0.3,
            'left_shoulder_roll':    2.7,
            'left_elbow_pitch':      0.4,
            'left_wrist_pitch':     -0.4,
            'neck_pitch': -0.2,
            'neck_yaw':   0.0,
        }, 0.4),
        # arms up, wrists tilted in (shake)
        _frame({
            'right_shoulder_pitch': -0.3,
            'right_shoulder_roll':   2.5,
            'right_elbow_pitch':     0.3,
            'right_wrist_pitch':    -0.4,
            'left_shoulder_pitch':  -0.3,
            'left_shoulder_roll':    2.5,
            'left_elbow_pitch':      0.3,
            'left_wrist_pitch':      0.4,
            'neck_pitch': -0.15,
            'neck_yaw':   0.1,
        }, 0.4),
        # arms up, wrists out again
        _frame({
            'right_shoulder_pitch': -0.3,
            'right_shoulder_roll':   2.7,
            'right_elbow_pitch':     0.4,
            'right_wrist_pitch':     0.4,
            'left_shoulder_pitch':  -0.3,
            'left_shoulder_roll':    2.7,
            'left_elbow_pitch':      0.4,
            'left_wrist_pitch':     -0.4,
            'neck_pitch': -0.2,
            'neck_yaw':  -0.1,
        }, 0.4),
        # arms spread wide to sides (jazz hands moment)
        _frame({
            'right_shoulder_pitch':  0.5,
            'right_shoulder_roll':   1.8,
            'right_elbow_pitch':     0.0,
            'right_wrist_pitch':     0.3,
            'left_shoulder_pitch':  -0.5,
            'left_shoulder_roll':    1.8,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':     -0.3,
            'neck_pitch': 0.0,
            'neck_yaw':   0.0,
        }, 0.5),
        _frame(NEUTRAL_POSE, 0.6),
    ],

    # ── wave ─────────────────────────────────────────────────────
    # Alternating arm raises — friendly crowd wave.
    'wave': [
        # right arm rises, left stays low
        _frame({
            'right_shoulder_pitch': -0.1,
            'right_shoulder_roll':   2.1,
            'right_elbow_pitch':     0.3,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    0.3,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.05,
            'neck_yaw':   0.2,
        }, 0.5),
        # both arms at mid height — transition
        _frame({
            'right_shoulder_pitch': 0.0,
            'right_shoulder_roll':  1.4,
            'right_elbow_pitch':    0.2,
            'right_wrist_pitch':    0.0,
            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   1.4,
            'left_elbow_pitch':     0.2,
            'left_wrist_pitch':     0.0,
            'neck_pitch': 0.05,
            'neck_yaw':   0.0,
        }, 0.4),
        # left arm rises, right comes down
        _frame({
            'right_shoulder_pitch':  0.0,
            'right_shoulder_roll':   0.3,
            'right_elbow_pitch':     0.0,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':  -0.1,
            'left_shoulder_roll':    2.1,
            'left_elbow_pitch':      0.3,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.05,
            'neck_yaw':  -0.2,
        }, 0.5),
        # both arms at mid height — transition
        _frame({
            'right_shoulder_pitch': 0.0,
            'right_shoulder_roll':  1.4,
            'right_elbow_pitch':    0.2,
            'right_wrist_pitch':    0.0,
            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   1.4,
            'left_elbow_pitch':     0.2,
            'left_wrist_pitch':     0.0,
            'neck_pitch': 0.05,
            'neck_yaw':   0.0,
        }, 0.4),
        # right arm rises again
        _frame({
            'right_shoulder_pitch': -0.1,
            'right_shoulder_roll':   2.1,
            'right_elbow_pitch':     0.3,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    0.3,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.05,
            'neck_yaw':   0.2,
        }, 0.5),
        _frame(NEUTRAL_POSE, 0.6),
    ],

    # ── groove ───────────────────────────────────────────────────
    # Funky pumping rhythm — arms sweep forward and back alternately.
    'groove': [
        # right arm sweeps forward-up, left sweeps back-down
        _frame({
            'right_shoulder_pitch':  0.3,
            'right_shoulder_roll':   2.0,
            'right_elbow_pitch':    -0.5,
            'right_wrist_pitch':     0.2,
            'left_shoulder_pitch':  -0.3,
            'left_shoulder_roll':    0.5,
            'left_elbow_pitch':      0.4,
            'left_wrist_pitch':     -0.2,
            'neck_pitch': 0.1,
            'neck_yaw':   0.3,
        }, 0.4),
        # left arm sweeps forward-up, right sweeps back-down
        _frame({
            'right_shoulder_pitch': -0.3,
            'right_shoulder_roll':   0.5,
            'right_elbow_pitch':     0.4,
            'right_wrist_pitch':    -0.2,
            'left_shoulder_pitch':   0.3,
            'left_shoulder_roll':    2.0,
            'left_elbow_pitch':     -0.5,
            'left_wrist_pitch':      0.2,
            'neck_pitch': 0.1,
            'neck_yaw':  -0.3,
        }, 0.4),
        # both arms pump forward together
        _frame({
            'right_shoulder_pitch':  0.0,
            'right_shoulder_roll':   1.8,
            'right_elbow_pitch':    -0.4,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    1.8,
            'left_elbow_pitch':     -0.4,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.15,
            'neck_yaw':   0.0,
        }, 0.35),
        # both arms pull back
        _frame({
            'right_shoulder_pitch':  0.0,
            'right_shoulder_roll':   0.8,
            'right_elbow_pitch':     0.5,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    0.8,
            'left_elbow_pitch':      0.5,
            'left_wrist_pitch':      0.0,
            'neck_pitch': -0.05,
            'neck_yaw':   0.0,
        }, 0.35),
        # repeat the alternating sweep
        _frame({
            'right_shoulder_pitch':  0.3,
            'right_shoulder_roll':   2.0,
            'right_elbow_pitch':    -0.5,
            'right_wrist_pitch':     0.2,
            'left_shoulder_pitch':  -0.3,
            'left_shoulder_roll':    0.5,
            'left_elbow_pitch':      0.4,
            'left_wrist_pitch':     -0.2,
            'neck_pitch': 0.1,
            'neck_yaw':   0.3,
        }, 0.4),
        _frame({
            'right_shoulder_pitch': -0.3,
            'right_shoulder_roll':   0.5,
            'right_elbow_pitch':     0.4,
            'right_wrist_pitch':    -0.2,
            'left_shoulder_pitch':   0.3,
            'left_shoulder_roll':    2.0,
            'left_elbow_pitch':     -0.5,
            'left_wrist_pitch':      0.2,
            'neck_pitch': 0.1,
            'neck_yaw':  -0.3,
        }, 0.4),
        _frame(NEUTRAL_POSE, 0.5),
    ],

    # ─────────────────────────────────────────────────────────────
    # Common Interaction Commands
    # ─────────────────────────────────────────────────────────────

    # ── bye ──────────────────────────────────────────────────────
    # Right arm raised, hand waves side to side three times.
    'bye': [
        _frame({
            'right_shoulder_pitch': 0.0,
            'right_shoulder_roll':  2.2,
            'right_elbow_pitch':    0.4,
            'right_wrist_pitch':    0.0,
            'left_shoulder_pitch':  0.0,
            'left_shoulder_roll':   0.0,
            'left_elbow_pitch':     0.0,
            'left_wrist_pitch':     0.0,
            'neck_pitch': 0.05, 'neck_yaw': 0.15,
        }, 0.55),
        _frame({
            'right_shoulder_pitch': 0.0, 'right_shoulder_roll': 2.2,
            'right_elbow_pitch': 0.4,    'right_wrist_pitch': -0.45,
            'left_shoulder_pitch': 0.0,  'left_shoulder_roll': 0.0,
            'left_elbow_pitch': 0.0,     'left_wrist_pitch': 0.0,
            'neck_pitch': 0.05, 'neck_yaw': 0.1,
        }, 0.25),
        _frame({
            'right_shoulder_pitch': 0.0, 'right_shoulder_roll': 2.2,
            'right_elbow_pitch': 0.4,    'right_wrist_pitch':  0.45,
            'left_shoulder_pitch': 0.0,  'left_shoulder_roll': 0.0,
            'left_elbow_pitch': 0.0,     'left_wrist_pitch': 0.0,
            'neck_pitch': 0.05, 'neck_yaw': 0.1,
        }, 0.25),
        _frame({
            'right_shoulder_pitch': 0.0, 'right_shoulder_roll': 2.2,
            'right_elbow_pitch': 0.4,    'right_wrist_pitch': -0.45,
            'left_shoulder_pitch': 0.0,  'left_shoulder_roll': 0.0,
            'left_elbow_pitch': 0.0,     'left_wrist_pitch': 0.0,
            'neck_pitch': 0.05, 'neck_yaw': 0.1,
        }, 0.25),
        _frame({
            'right_shoulder_pitch': 0.0, 'right_shoulder_roll': 2.2,
            'right_elbow_pitch': 0.4,    'right_wrist_pitch':  0.45,
            'left_shoulder_pitch': 0.0,  'left_shoulder_roll': 0.0,
            'left_elbow_pitch': 0.0,     'left_wrist_pitch': 0.0,
            'neck_pitch': 0.05, 'neck_yaw': 0.1,
        }, 0.25),
        _frame({
            'right_shoulder_pitch': 0.0, 'right_shoulder_roll': 2.2,
            'right_elbow_pitch': 0.4,    'right_wrist_pitch': -0.45,
            'left_shoulder_pitch': 0.0,  'left_shoulder_roll': 0.0,
            'left_elbow_pitch': 0.0,     'left_wrist_pitch': 0.0,
            'neck_pitch': 0.05, 'neck_yaw': 0.1,
        }, 0.25),
        _frame(NEUTRAL_POSE, 0.5),
    ],

    # ── welcome ──────────────────────────────────────────────────
    # Both arms spread wide with a slight forward bow.
    'welcome': [
        _frame({
            'right_shoulder_pitch':  0.5,
            'right_shoulder_roll':   1.5,
            'right_elbow_pitch':    -0.2,
            'right_wrist_pitch':    -0.2,
            'left_shoulder_pitch':  -0.5,
            'left_shoulder_roll':    1.5,
            'left_elbow_pitch':     -0.2,
            'left_wrist_pitch':      0.2,
            'neck_pitch': 0.05, 'neck_yaw': 0.0,
        }, 1.1),
        # slight bow
        _frame({
            'right_shoulder_pitch':  0.5,
            'right_shoulder_roll':   1.5,
            'right_elbow_pitch':    -0.2,
            'right_wrist_pitch':    -0.2,
            'left_shoulder_pitch':  -0.5,
            'left_shoulder_roll':    1.5,
            'left_elbow_pitch':     -0.2,
            'left_wrist_pitch':      0.2,
            'neck_pitch': 0.3, 'neck_yaw': 0.0,
        }, 0.5),
        _frame(NEUTRAL_POSE, 0.6),
    ],

    # ── please ───────────────────────────────────────────────────
    # ASL please: flat open hand makes two circles over the chest.
    'please': [
        _frame({
            'right_shoulder_pitch':   0.0,
            'right_shoulder_roll':    1.1,
            'right_elbow_pitch':      1.3,
            'right_wrist_pitch':      0.0,
            'left_shoulder_pitch':    0.0,
            'left_shoulder_roll':     0.0,
            'left_elbow_pitch':       0.0,
            'left_wrist_pitch':       0.0,
            'neck_pitch': 0.1, 'neck_yaw': 0.0,
        }, 0.45),
        _frame({
            'right_shoulder_pitch': -0.15, 'right_shoulder_roll': 1.2,
            'right_elbow_pitch':     1.15,  'right_wrist_pitch':  0.1,
            'left_shoulder_pitch':   0.0,   'left_shoulder_roll': 0.0,
            'left_elbow_pitch':      0.0,   'left_wrist_pitch':   0.0,
            'neck_pitch': 0.1, 'neck_yaw': 0.0,
        }, 0.28),
        _frame({
            'right_shoulder_pitch':  0.15, 'right_shoulder_roll': 1.0,
            'right_elbow_pitch':     1.4,   'right_wrist_pitch': -0.1,
            'left_shoulder_pitch':   0.0,   'left_shoulder_roll': 0.0,
            'left_elbow_pitch':      0.0,   'left_wrist_pitch':   0.0,
            'neck_pitch': 0.1, 'neck_yaw': 0.0,
        }, 0.28),
        _frame({
            'right_shoulder_pitch': -0.15, 'right_shoulder_roll': 1.2,
            'right_elbow_pitch':     1.15,  'right_wrist_pitch':  0.1,
            'left_shoulder_pitch':   0.0,   'left_shoulder_roll': 0.0,
            'left_elbow_pitch':      0.0,   'left_wrist_pitch':   0.0,
            'neck_pitch': 0.1, 'neck_yaw': 0.0,
        }, 0.28),
        _frame({
            'right_shoulder_pitch':  0.15, 'right_shoulder_roll': 1.0,
            'right_elbow_pitch':     1.4,   'right_wrist_pitch': -0.1,
            'left_shoulder_pitch':   0.0,   'left_shoulder_roll': 0.0,
            'left_elbow_pitch':      0.0,   'left_wrist_pitch':   0.0,
            'neck_pitch': 0.1, 'neck_yaw': 0.0,
        }, 0.28),
        _frame(NEUTRAL_POSE, 0.5),
    ],

    # ── thanks ───────────────────────────────────────────────────
    # ASL thank you: flat hand near chin, sweeps forward and down.
    'thanks': [
        _frame({
            'right_shoulder_pitch': -0.2,
            'right_shoulder_roll':   2.1,
            'right_elbow_pitch':     0.9,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    0.0,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.1, 'neck_yaw': 0.0,
        }, 0.65),
        _frame({
            'right_shoulder_pitch':  0.0,
            'right_shoulder_roll':   1.5708,
            'right_elbow_pitch':     0.2,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    0.0,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.05, 'neck_yaw': 0.0,
        }, 0.5),
        _frame(NEUTRAL_POSE, 0.5),
    ],

    # ── yes ──────────────────────────────────────────────────────
    # Head nods twice, arms in signing-ready position.
    'yes': [
        _frame({**SIGNING_POSE, 'neck_pitch':  0.4, 'neck_yaw': 0.0}, 0.28),
        _frame({**SIGNING_POSE, 'neck_pitch':  0.05, 'neck_yaw': 0.0}, 0.22),
        _frame({**SIGNING_POSE, 'neck_pitch':  0.4, 'neck_yaw': 0.0}, 0.28),
        _frame({**SIGNING_POSE, 'neck_pitch':  0.05, 'neck_yaw': 0.0}, 0.22),
        _frame(NEUTRAL_POSE, 0.4),
    ],

    # ── no ───────────────────────────────────────────────────────
    # Head shakes twice, arms in signing-ready position.
    'no': [
        _frame({**SIGNING_POSE, 'neck_pitch': 0.0, 'neck_yaw':  0.4}, 0.28),
        _frame({**SIGNING_POSE, 'neck_pitch': 0.0, 'neck_yaw': -0.4}, 0.28),
        _frame({**SIGNING_POSE, 'neck_pitch': 0.0, 'neck_yaw':  0.4}, 0.28),
        _frame({**SIGNING_POSE, 'neck_pitch': 0.0, 'neck_yaw': -0.4}, 0.28),
        _frame(NEUTRAL_POSE, 0.4),
    ],

    # ── sorry ────────────────────────────────────────────────────
    # ASL sorry: closed fist makes two circles over the chest.
    'sorry': [
        _frame({
            'right_shoulder_pitch':   0.0,
            'right_shoulder_roll':    1.1,
            'right_elbow_pitch':      1.3,
            'right_wrist_pitch':      0.0,
            'left_shoulder_pitch':    0.0,
            'left_shoulder_roll':     0.0,
            'left_elbow_pitch':       0.0,
            'left_wrist_pitch':       0.0,
            'neck_pitch': 0.15, 'neck_yaw': 0.0,
        }, 0.45),
        _frame({
            'right_shoulder_pitch': -0.15, 'right_shoulder_roll': 1.2,
            'right_elbow_pitch':     1.15,  'right_wrist_pitch':  0.0,
            'left_shoulder_pitch':   0.0,   'left_shoulder_roll': 0.0,
            'left_elbow_pitch':      0.0,   'left_wrist_pitch':   0.0,
            'neck_pitch': 0.15, 'neck_yaw': 0.0,
        }, 0.28),
        _frame({
            'right_shoulder_pitch':  0.15, 'right_shoulder_roll': 1.0,
            'right_elbow_pitch':     1.4,   'right_wrist_pitch':  0.0,
            'left_shoulder_pitch':   0.0,   'left_shoulder_roll': 0.0,
            'left_elbow_pitch':      0.0,   'left_wrist_pitch':   0.0,
            'neck_pitch': 0.15, 'neck_yaw': 0.0,
        }, 0.28),
        _frame({
            'right_shoulder_pitch': -0.15, 'right_shoulder_roll': 1.2,
            'right_elbow_pitch':     1.15,  'right_wrist_pitch':  0.0,
            'left_shoulder_pitch':   0.0,   'left_shoulder_roll': 0.0,
            'left_elbow_pitch':      0.0,   'left_wrist_pitch':   0.0,
            'neck_pitch': 0.15, 'neck_yaw': 0.0,
        }, 0.28),
        _frame({
            'right_shoulder_pitch':  0.15, 'right_shoulder_roll': 1.0,
            'right_elbow_pitch':     1.4,   'right_wrist_pitch':  0.0,
            'left_shoulder_pitch':   0.0,   'left_shoulder_roll': 0.0,
            'left_elbow_pitch':      0.0,   'left_wrist_pitch':   0.0,
            'neck_pitch': 0.15, 'neck_yaw': 0.0,
        }, 0.28),
        _frame(NEUTRAL_POSE, 0.5),
    ],

    # ── come ─────────────────────────────────────────────────────
    # Arm extends forward then beckons twice.
    'come': [
        _frame({
            'right_shoulder_pitch':  0.0,
            'right_shoulder_roll':   1.5708,
            'right_elbow_pitch':     0.0,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    0.0,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.05, 'neck_yaw': 0.2,
        }, 0.5),
        _frame({
            'right_shoulder_pitch':  0.0,
            'right_shoulder_roll':   1.5708,
            'right_elbow_pitch':    -1.0,
            'right_wrist_pitch':     0.3,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    0.0,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.05, 'neck_yaw': 0.2,
        }, 0.35),
        _frame({
            'right_shoulder_pitch':  0.0,
            'right_shoulder_roll':   1.5708,
            'right_elbow_pitch':     0.0,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    0.0,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.05, 'neck_yaw': 0.2,
        }, 0.3),
        _frame({
            'right_shoulder_pitch':  0.0,
            'right_shoulder_roll':   1.5708,
            'right_elbow_pitch':    -1.0,
            'right_wrist_pitch':     0.3,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    0.0,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.05, 'neck_yaw': 0.2,
        }, 0.35),
        _frame(NEUTRAL_POSE, 0.5),
    ],

    # ── stop ─────────────────────────────────────────────────────
    # Right arm raised with palm facing out — halt signal.
    'stop': [
        _frame({
            'right_shoulder_pitch': -0.2,
            'right_shoulder_roll':   2.0,
            'right_elbow_pitch':     0.2,
            'right_wrist_pitch':    -0.5,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    0.0,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.05, 'neck_yaw': 0.0,
        }, 1.2),
        _frame(NEUTRAL_POSE, 0.5),
    ],

    # ── good ─────────────────────────────────────────────────────
    # ASL good: flat hand near chin, sweeps forward and down.
    'good': [
        _frame({
            'right_shoulder_pitch': -0.2,
            'right_shoulder_roll':   2.0,
            'right_elbow_pitch':     0.7,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    0.0,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.1, 'neck_yaw': 0.0,
        }, 0.6),
        _frame({
            'right_shoulder_pitch':  0.1,
            'right_shoulder_roll':   1.4,
            'right_elbow_pitch':     0.3,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    0.0,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.05, 'neck_yaw': 0.0,
        }, 0.5),
        _frame(NEUTRAL_POSE, 0.5),
    ],

    # ── disco ────────────────────────────────────────────────────
    # Classic Saturday-Night-Fever alternating diagonal point.
    'disco': [
        # right arm points up-right diagonal, left arm points down-left
        _frame({
            'right_shoulder_pitch': -0.6,
            'right_shoulder_roll':   2.3,
            'right_elbow_pitch':     0.0,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.6,
            'left_shoulder_roll':    0.4,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': -0.1,
            'neck_yaw':   0.4,
        }, 0.45),
        # arms cross through center
        _frame({
            'right_shoulder_pitch':  0.0,
            'right_shoulder_roll':   1.5708,
            'right_elbow_pitch':     0.0,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    1.5708,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.0,
            'neck_yaw':   0.0,
        }, 0.3),
        # left arm points up-left diagonal, right arm points down-right
        _frame({
            'right_shoulder_pitch':  0.6,
            'right_shoulder_roll':   0.4,
            'right_elbow_pitch':     0.0,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':  -0.6,
            'left_shoulder_roll':    2.3,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': -0.1,
            'neck_yaw':  -0.4,
        }, 0.45),
        # arms cross through center
        _frame({
            'right_shoulder_pitch':  0.0,
            'right_shoulder_roll':   1.5708,
            'right_elbow_pitch':     0.0,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.0,
            'left_shoulder_roll':    1.5708,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': 0.0,
            'neck_yaw':   0.0,
        }, 0.3),
        # right arm up-right again (repeat)
        _frame({
            'right_shoulder_pitch': -0.6,
            'right_shoulder_roll':   2.3,
            'right_elbow_pitch':     0.0,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':   0.6,
            'left_shoulder_roll':    0.4,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': -0.1,
            'neck_yaw':   0.4,
        }, 0.45),
        # left arm up-left again
        _frame({
            'right_shoulder_pitch':  0.6,
            'right_shoulder_roll':   0.4,
            'right_elbow_pitch':     0.0,
            'right_wrist_pitch':     0.0,
            'left_shoulder_pitch':  -0.6,
            'left_shoulder_roll':    2.3,
            'left_elbow_pitch':      0.0,
            'left_wrist_pitch':      0.0,
            'neck_pitch': -0.1,
            'neck_yaw':  -0.4,
        }, 0.45),
        _frame(NEUTRAL_POSE, 0.6),
    ],
}

# ─────────────────────────────────────────────────────────────
# Hand Shape Overrides
# ─────────────────────────────────────────────────────────────
HAND_SHAPE_OVERRIDES = {
    'hello': {'right': 'hello', 'left': 'neutral'},
    'boy':   {'right': 'boy',   'left': 'neutral'},
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

    # interaction commands
    'bye':      {'right': 'hello',   'left': 'neutral'},  # open hand wave
    'welcome':  {'right': 'b',       'left': 'b'},         # flat open hands
    'please':   {'right': 'b',       'left': 'neutral'},   # flat hand on chest
    'thanks':   {'right': 'b',       'left': 'neutral'},   # flat hand from chin
    'yes':      {'right': 'a',       'left': 'neutral'},   # closed fist nod
    'no':       {'right': 'neutral', 'left': 'neutral'},
    'sorry':    {'right': 'a',       'left': 'neutral'},   # closed fist on chest
    'come':     {'right': 'neutral', 'left': 'neutral'},   # open beckoning hand
    'stop':     {'right': 'b',       'left': 'neutral'},   # flat palm out
    'good':     {'right': 'b',       'left': 'neutral'},   # flat hand from chin

    # dance moves
    'robot':     {'right': 'neutral', 'left': 'neutral'},
    'celebrate': {'right': 'neutral', 'left': 'neutral'},
    'wave':      {'right': 'neutral', 'left': 'neutral'},
    'groove':    {'right': 'neutral', 'left': 'neutral'},
    'disco':     {'right': 'neutral', 'left': 'neutral'},

    # alphabet
    'a':     {'right': 'a',     'left': 'neutral'},
    'b':     {'right': 'b',     'left': 'neutral'},
    'c':     {'right': 'c',     'left': 'neutral'},
    'd':     {'right': 'd',     'left': 'neutral'},
    'e':     {'right': 'e',     'left': 'neutral'},
    'f':     {'right': 'f',     'left': 'neutral'},
    'g':     {'right': 'g',     'left': 'neutral'},
    'h':     {'right': 'h',     'left': 'neutral'},
    'i':     {'right': 'i',     'left': 'neutral'},
    'j':     {'right': 'j',     'left': 'neutral'},
    'k':     {'right': 'k',     'left': 'neutral'},
    'l':     {'right': 'l',     'left': 'neutral'},
    'm':     {'right': 'm',     'left': 'neutral'},
    'n':     {'right': 'n',     'left': 'neutral'},
    'o':     {'right': 'o',     'left': 'neutral'},
    'p':     {'right': 'p',     'left': 'neutral'},
    'q':     {'right': 'q',     'left': 'neutral'},
    'r':     {'right': 'r',     'left': 'neutral'},
    's':     {'right': 's',     'left': 'neutral'},
    't':     {'right': 't',     'left': 'neutral'},
    'u':     {'right': 'u',     'left': 'neutral'},
    'v':     {'right': 'v',     'left': 'neutral'},
    'w':     {'right': 'w',     'left': 'neutral'},
    'x':     {'right': 'x',     'left': 'neutral'},
    'y':     {'right': 'y',     'left': 'neutral'},
    'z':     {'right': 'z',     'left': 'neutral'},
}