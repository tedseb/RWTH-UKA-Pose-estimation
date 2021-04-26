
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

NUMBER_OF_COMPARATOR_THREADS = 1

REDIS_MAXIMUM_QUEUE_SIZE = 100
REDIS_MAXIMUM_PAST_QUEUE_SIZE = 100
STATION_QUEUE_SIZE_MINIMUM = 0

REDIS_USER_STATE_SENDING_QUEUE_NAME = "user_state_sending_queue"
REDIS_USER_INFO_SENDING_QUEUE_NAME = "user_correction_sending_queue"
REDIS_LOAD_BALANCER_LIST_KEY = "spot_queue_load_balancer_list"

QUEUEING_USER_STATE_QUEUE_SIZE_MAX = 100
QUEUEING_USER_INFO_QUEUE_SIZE_MAX = 100

EXERCISE_DATA_LRU_CACHE_SIZE = 1000

assert STATION_QUEUE_SIZE_MINIMUM <= REDIS_MAXIMUM_QUEUE_SIZE

# Incase other people want to use the same redis database, better use a prefix for keys so our data does not collide
REDIS_GENERAL_PREFIX = "ComparingSystem:"
REDIS_SPOT_QUEUE_POSTFIX = ":queue"
REDIS_SPOT_PAST_QUEUE_POSTFIX = ":queue_past"
REDIS_SPOT_INFO_POSTFIX = ":info"
REDIS_SPOT_STATE_POSTFIX = ":state"
REDIS_SPOT_EXERCISE_POSTFIX = ":exercise"

SEND_CORRETIONS = False

# Comparing system talks lots if this is set to true
HIGH_VERBOSITY = False

# ALGORITHMIC COMFIGURATION

# 0 means that user has to reach angle at least as high/low as performed by expert
# 1 means that the boundaries are swapped (and makes no sense)
REDUCED_RANGE_OF_MOTION_TOLERANCE_LOWER = 0.3
REDUCED_RANGE_OF_MOTION_TOLERANCE_HIGHER = 0.3
REDUCED_RANGE_OF_MOTION_TOLERANCE_MINIMAL_DISTANCE_HIGHER_LOWER = 0.3

assert REDUCED_RANGE_OF_MOTION_TOLERANCE_LOWER + REDUCED_RANGE_OF_MOTION_TOLERANCE_HIGHER <= 1 - REDUCED_RANGE_OF_MOTION_TOLERANCE_MINIMAL_DISTANCE_HIGHER_LOWER