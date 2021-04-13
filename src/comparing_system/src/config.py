
"""
This file contains some parameters to our ComparingNode, such as ROS Topic names and queueing limits.
"""

DEFAULT_JOINT_ADAPTER = 'src.joint_adapters.spin'

ROS_TOPIC_USER_EXERCISE_STATES = 'user_state'
ROS_TOPIC_USER_CORRECTIONS = 'user_correction'

ROS_JOINTS_TOPIC = 'fused_skelleton' # Shawan's topic choice
ROS_EXPERT_SYSTEM_UPDATE_TOPIC = 'exercises'
ROS_SKELLETON_DEFINITION = 'skelleton_name'

NUMBER_OF_COMPARATOR_THREADS = 20

REDIS_MAXIMUM_QUEUE_SIZE = 100
REDIS_MAXIMUM_PAST_QUEUE_SIZE = 100
STATION_QUEUE_SIZE_MINIMUM = 0

REDIS_USER_STATE_SENDING_QUEUE_NAME = "user_state_sending_queue"
REDIS_USER_INFO_SENDING_QUEUE_NAME = "user_correction_sending_queue"
REDIS_LOAD_BALANCER_LIST_KEY = "spot_queue_load_balancer_list"

QUEUEING_USER_STATE_QUEUE_SIZE_MAX = 100
QUEUEING_USER_INFO_QUEUE_SIZE_MAX = 100

assert STATION_QUEUE_SIZE_MINIMUM <= REDIS_MAXIMUM_QUEUE_SIZE

# Incase other people want to use the same redis database, better use a prefix for keys so our data does not collide
REDIS_GENERAL_PREFIX = "ComparingSystem:"
REDIS_SPOT_QUEUE_POSTFIX = ":queue"
REDIS_SPOT_PAST_QUEUE_POSTFIX = ":queue_past"
REDIS_SPOT_INFO_POSTFIX = ":info"

LEGACY_COMPARING = True
SEND_CORRETIONS = False

HIGH_VERBOSITY = False