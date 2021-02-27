
"""
This file contains some parameters to our ComparingNode, such as ROS Topic names and queueing limits.
"""

DEFAULT_JOINT_ADAPTER = 'src.joint_adapters.spin'

ROS_TOPIC_USER_EXERCISE_STATES = '/user_states'
ROS_TOPIC_USER_CORRECTIONS = '/user_corrections'

ROS_JOINTS_TOPIC = 'fused_skelleton' # Shawan's topic choice
ROS_EXERCISES_CHANGE_TOPIC = 'changeme' # TODO: Change this from tamer
ROS_SKELLETON_DEFINITION = 'skelleton_name'

NUMBER_OF_COMPARATOR_THREADS = 1

REDIS_MAXIMUM_QUEUE_SIZE = 100
REDIS_MAXIMUM_PAST_QUEUE_SIZE = 100
STATION_QUEUE_SIZE_MINIMUM = 0

REDIS_USER_STATE_SENDING_QUEUE_NAME = "user_state_sending_queue"
REDIS_USER_INFO_SENDING_QUEUE_NAME = "user_info_sending_queue"

if STATION_QUEUE_SIZE_MINIMUM >= REDIS_MAXIMUM_QUEUE_SIZE:
    raise Exception("The minimum spot queue size can not be longer than the maximum queue size of the spot's Redis queue.")
