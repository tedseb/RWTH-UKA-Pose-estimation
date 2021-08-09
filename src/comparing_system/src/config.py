
"""
This file contains some parameters to our ComparingNode, such as ROS Topic names and queueing limits.
"""

# GENERAL COMPARING SYSTEM CONFIGURATION (not algorithmic)
DEFAULT_JOINT_ADAPTER = 'src.joint_adapters.spin'

ROS_TOPIC_USER_EXERCISE_STATES = 'user_state'
ROS_TOPIC_USER_CORRECTIONS = 'user_correction'

ROS_JOINTS_TOPIC = 'fused_skelleton' # Shawan's topic choice
ROS_EXPERT_SYSTEM_UPDATE_TOPIC = 'exercises'
ROS_SKELLETON_DEFINITION = 'skelleton_name'

REDIS_MAXIMUM_QUEUE_SIZE = 300
REDIS_QUEUE_SIZE_PANIC_BOUNDARY = 100
REDIS_MAXIMUM_PAST_QUEUE_SIZE = 100
REDIS_KEY_SEPARATOR = ":"

REDIS_USER_STATE_SENDING_QUEUE_NAME = "user_state_sending_queue"
REDIS_USER_INFO_SENDING_QUEUE_NAME = "user_correction_sending_queue"
REDIS_LOAD_BALANCER_SORTED_SET_KEY = "spot_queue_load_balancer_list"
REDIS_LOAD_BALANCER_ACTIVE_COMPARATORS_SORTED_SET_KEY = "spot_queue_load_balancer_active_comparators"  # Currently not in use

QUEUEING_USER_STATE_QUEUE_SIZE_MAX = 100
QUEUEING_USER_INFO_QUEUE_SIZE_MAX = 100

EXERCISE_DATA_LRU_CACHE_SIZE = 1000

# Incase other people want to use the same redis database, better use a prefix for keys so our data does not collide
REDIS_GENERAL_PREFIX = "comparing_system"
REDIS_SPOT_QUEUE_POSTFIX = "pose_queue"
REDIS_SPOT_PAST_QUEUE_POSTFIX = "pose_queue_past"
REDIS_SPOT_INFO_POSTFIX = "spot_info"
REDIS_SPOT_STATE_POSTFIX = "spot_state"
REDIS_SPOT_EXERCISE_POSTFIX = "exercise"
REDIS_SPOT_FEATURE_PROGRESSION_POSTFIX = "feature_progression"
REDIS_SPOT_RESAMPLED_FEATURES_POSTFIX = "resampled_features"

SEND_CORRETIONS = False

# Comparing system talks lots if this is set to true
HIGH_VERBOSITY = True

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

AVERAGE_SKELETON_QUEUE_LENGTH = 3