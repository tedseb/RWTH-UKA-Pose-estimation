"""
In order to implement extreme low latency, high throughput and scalable queueing, we may want to employ different queues.
This file defines the SpotQueueInterface and MessageQueueInterface alongside possible implementations of said interfaces. 
The queueing should be trasparent to the Comparator and the ComparingNode.
"""

import threading
import redis
import rospy as rp
import yaml

from src.config import *

from typing import Tuple, Any

class QueueingException(Exception):
    pass

class QueueEmpty(QueueingException):
    pass

class QueueTimeout(QueueingException):
    pass


class SpotQueueInterface:
    """
    We need to queue different data for our spots.
    Said data is primarely defined by the joints and their timestamps.
    We assume that every spot, at any given time, has only one active user.
    Therefore, we assume that we can sort data from multiple views into the same spot-queue, providing their timestamps
    """
    def __init__(self):
        raise NotImplementedError("This is an interface and shold not be called directly")

    def dequeue(self, spot_key: str) -> Tuple[dict, list, list, list]:
        """
        Dequeue spot data from queues and return it as a tuple:
        Return spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list
        """
        raise NotImplementedError("This is an interface and shold not be called directly")

    def enqueue(self, spot_key: str, data: Any) -> None:
        raise NotImplementedError("This is an interface and shold not be called directly")

class MessageQueueInterface:
    # TODO: Implement
    def __init__(self):
        raise NotImplementedError("This is an interface and shold not be called directly")
    
    def enqueue(self, key: str, message: Any):
        raise NotImplementedError("This is an interface and shold not be called directly")

    def dequeue(self, key):
        raise NotImplementedError("This is an interface and shold not be called directly")

    def delete(self):
        raise NotImplementedError("This is an interface and shold not be called directly")


class RedisMessageQueue():
    # TODO: Implement
    def __init__(self, redis_connection: redis.Redis):
        self.redis_connection = redis_connection

    def enqueue(self, key: str, message: Any):
        raise NotImplementedError

    def dequeue(self, key):
        raise NotImplementedError

    def delete(self, key):
        raise NotImplementedError


class RedisSpotQueueInterface(SpotQueueInterface):
    """
    This class helps with enqueue and dequeueing from and to a redis database enqueue and dequeue data that the comparator needs with Redis.

    Our Redis FIFO Queues that contain joints that are produced by whatever AI our System uses.

    * The 'queue' for a spot is expected to contain a list of of dictionaries '{'bodyParts': <Bodyparts>, 'time_stamp': <header.stamp>}'.
    These dictionaries represent data not yet processed by the comparator, or 'the future'.
    * The 'past queue' for a spot is expected to contains the dictionaries already processed, or 'the past'.
    * The info for a spot contains additional information like timing data in a dictionary.
    """

    def __init__(self, redis_connection: redis.Redis, dequeue_semaphore: threading.Semaphore):
        self.redis_connection = redis_connection
        # Dequeueing consists of several operations. Until we move to a distribuetd system, we can use a semaphore to guarantee correct queuing.
        # Redis has distributed locks: https://redis.io/topics/distlock
        self.dequeue_semaphore = dequeue_semaphore

    def dequeue(self, spot_key: str) -> Tuple[dict, list, list, list]:
        """
        Return spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list
        """
        # TODO: Investigate if these redis instructions can be optimized
        # Fetch all data that is needed for the comparison:

        redis_spot_queue_key, redis_spot_past_queue_key, redis_spot_info_key = generate_redis_key_names(spot_key)
        
        try:
            assert self.dequeue_semaphore.acquire(blocking=True, timeout=0.1)
        except AssertionError:
            raise QueueEmpty

        try:
            joints_with_timestamp_yaml = self.redis_connection.rpoplpush(redis_spot_queue_key, redis_spot_past_queue_key)
            assert joints_with_timestamp_yaml
        except (KeyError, AssertionError):
            # Supposingly, no message queue is holding any value (at the start of the system)
            raise QueueEmpty

        self.redis_connection.ltrim(redis_spot_past_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)

        future_joints_with_timestamp_list = self.redis_connection.lrange(redis_spot_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)
        past_joints_with_timestamp_list = self.redis_connection.lrange(redis_spot_past_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)

        self.dequeue_semaphore.release()

        joints_with_timestamp = yaml.load(joints_with_timestamp_yaml)
        
        spot_info_dict = yaml.load(self.redis_connection.get(redis_spot_info_key)) # TODO: Switch from using yaml to Rejson
        exercise = spot_info_dict['exercise']
        start_time = spot_info_dict['start_time']

        return spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list

    def enqueue(self, spot_key: str, data: Any) -> int:
        """ Return queue size """
        redis_spot_queue_key, _, _ = generate_redis_key_names(spot_key)
        queue_size = self.redis_connection.rpush(spot_key, yaml.dump(data)) # TODO: Use serialized bodyparts here (see above)

        if (queue_size >= REDIS_MAXIMUM_QUEUE_SIZE):
            rp.logerr("Maximum Queue size for spotID " + str(p.stationID) + " reached. Removing first element.")
            self.redis_connection.ltrim(redis_spot_queue_key, 0, REDIS_MAXIMUM_QUEUE_SIZE)

        return queue_size


def generate_redis_key_names(spot_key: str):
    """ 
    This method simplifies handling of key names.
    Return: redis_spot_queue_key, redis_spot_past_queue_key, redis_spot_info_key

    Data that is expected by the comparator is listed below.

    The redis keys and values are as follows:
        * Spot with ID N is:                <spot_key>               :(prefix for other data)
        * Queue for spot with ID N is       <spot_key:queue>         :Redis FIFO Queue
        * Past queue for spot with ID N is  <spot_key:queue_past>    :Redis FIFO Queue
        * Info for spot with ID N is        <spot_key:info>          :stringified YAML
    """
    redis_spot_queue_key = REDIS_GENERAL_PREFIX + spot_key + REDIS_SPOT_QUEUE_POSTFIX
    redis_spot_past_queue_key = REDIS_GENERAL_PREFIX + redis_spot_queue_key + REDIS_SPOT_PAST_QUEUE_POSTFIX
    redis_spot_info_key = REDIS_GENERAL_PREFIX + spot_key + REDIS_SPOT_INFO_POSTFIX

    return redis_spot_queue_key, redis_spot_past_queue_key, redis_spot_info_key