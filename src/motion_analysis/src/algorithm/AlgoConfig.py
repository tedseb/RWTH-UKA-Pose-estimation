
"""
This file contains some parameters to our algorithms.
"""

# ALGORITHMIC COMFIGURATION

# 0 means that user has to reach angle at least as high/low as performed by expert
# 1 means that the boundaries are swapped (and makes no sense)
REDUCED_RANGE_OF_MOTION_TOLERANCE_LOWER = 0.41
REDUCED_RANGE_OF_MOTION_TOLERANCE_HIGHER = 0.41
# REDUCED_RANGE_OF_MOTION_TOLERANCE_MINIMAL_DISTANCE_HIGHER_LOWER = 0.3
# assert REDUCED_RANGE_OF_MOTION_TOLERANCE_LOWER + REDUCED_RANGE_OF_MOTION_TOLERANCE_HIGHER <= 1 - REDUCED_RANGE_OF_MOTION_TOLERANCE_MINIMAL_DISTANCE_HIGHER_LOWER

# The final feature trajectory resolution is: FEATURE_TRAJECTORY_RESOLUTION_FACTOR  * range of motion of a single feature
FEATURE_TRAJECTORY_RESOLUTION_FACTOR = 0.1

# Message sender dequeueing timeout in seconds. A lower timeout lets us kill the threader quicker but results in higher resource consumption
SENDER_NODE_MESSAGE_DEQUEUEING_TIMEOUT = 1

# Feature difference elasticity - used to see if all features are close to each other in their reference trajectory
FEATURE_DIFFERENCE_ELASTICITY = 0.3

FEATURE_DIFFERENCE_MAX_QUEUE_LENGTH = 5

DIGITIZED_FEATURE_TRAJECTORY_MAX_MEMORY_SIZE = 30
FEATURE_TRAJECTORY_MAX_MEMORY_SIZE = 90
STATES_TRAJECTORY_MAX_MEMORY_SIZE = 30

# We have a resolution of ten steps between minimum and maximum of a trajectory. We remove 4 such steps of jittering, if found.
REMOVE_JITTER_RANGE = 5

# We use this to disable marking repetitions as bad if they seem weird if inputs are messy
ROBUST_COUNTING_MODE = True

JUMPY_PROGRESS_ALPHA = 3
JUMPY_PROGRESS_BETA = 4

# FPS for the GUI
GUI_FPS = 25

TRUST_REGION_FILTER_FACTOR = 2
MINIMAL_ALLOWED_MEAN_FEATURE_ALIGNMENT = 0.8

# We use this fading factor for a exponentially dying error
JOINT_DIFFERENCE_FADING_FACTOR = 0.8

# Indices of values in vectors
X = 0
Y = 1
Z = 2