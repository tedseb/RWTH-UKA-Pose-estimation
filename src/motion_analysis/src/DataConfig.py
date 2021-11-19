"""
This file contains some parameters to our data plane, such as ROS Topic names and queueing limits.
"""

# GENERAL CONFIGURATION (not algorithmic)

MONGO_DB_URI = "mongodb://mongoadmin:secret@localhost:27888/?authSource=admin"

DEFAULT_JOINT_ADAPTER = 'src.joint_adapters.metrabs'

ROS_TOPIC_USER_EXERCISE_STATES = 'user_state'
ROS_TOPIC_USER_CORRECTIONS = 'user_correction'

ROS_JOINTS_TOPIC = 'fused_skelleton' # Shawan's topic choice
ROS_STATION_USAGE_UPDATE_TOPIC = 'station_usage'
ROS_SKELLETON_DEFINITION = 'skelleton_name'

ROS_MOTION_ANALYSIS_INPUT_VISUALIZER_TOPIC = "motion_analysis_input_markers"
ROS_MOTION_ANALYSIS_REFERENCE_POSE_VISUALIZER_TOPIC = "motion_analysis_reference_prediction_markers"


REDIS_MAXIMUM_QUEUE_SIZE = 1000
REDIS_QUEUE_SIZE_PANIC_BOUNDARY = 200
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
REDIS_GENERAL_PREFIX = "motion_analysis"
REDIS_SPOT_QUEUE_POSTFIX = "pose_queue"
REDIS_SPOT_PAST_QUEUE_POSTFIX = "pose_queue_past"
REDIS_SPOT_INFO_POSTFIX = "spot_info"
REDIS_SPOT_FEATURES = "features"
REDIS_SPOT_EXERCISE_POSTFIX = "exercise"

SEND_CORRETIONS = False
JOINT_DIFFERENCE_FADING_FACTOR = 0.8

FEATURE_EXTRACTION_SERVER_PORT = 3002