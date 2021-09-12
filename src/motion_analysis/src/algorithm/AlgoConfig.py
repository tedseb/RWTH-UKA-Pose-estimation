
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

FEATURE_TRAJECTORY_MAX_MEMORY_SIZE = 100

REMOVE_JITTER_RANGE = 4

MESSY_INPUTS = True

# Indices of values in vectors
X = 0
Y = 1
Z = 2