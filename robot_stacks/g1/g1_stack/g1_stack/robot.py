"""What the G1 is, independent of what drives it.

Joint counts and the layout of the robot's own state vector. Nothing about any
policy, controller or task belongs here.
"""

# bridge/robot_bridge/params/G1_config.yaml: legs (12), waist (3),
# left arm (7), right arm (7).
NUM_BODY_JOINTS = 29

# The arm tail of that ordering. No remapping: arms are the last 14 of the 29.
ARM_SLICE = slice(15, 29)
NUM_ARM_JOINTS = 14

# Dex3-1, 7 joints per hand. Concatenation is left then right.
# UNCONFIRMED: that the order the two hands are *read* off DDS matches. Move one
# left-hand finger and watch which half of the 14-vector changes.
NUM_HAND_JOINTS_PER_HAND = 7
NUM_HAND_JOINTS = 14
