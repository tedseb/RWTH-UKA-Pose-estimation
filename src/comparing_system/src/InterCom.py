"""
In order to implement extreme low latency, high throughput and scalable queueing, we may want to employ different queues.
This file defines the SpotQueueInterface and MessageQueueInterface alongside possible implementations of said interfaces. 
The queueing should be trasparent to the Comparator and the ComparingNode.
"""

import threading
import redis
import rospy as rp
import yaml
import msgpack
from traceback import print_exc
from collections import OrderedDict
from abc import ABC, abstractmethod
import hashlib

from typing import Tuple, Any

from src.config import *

redis_connection_pool = redis.ConnectionPool(host='localhost', port=5678, db=0)

class QueueingException(Exception):
    pass


class QueueEmpty(QueueingException):
    pass


class QueueTimeout(QueueingException):
    pass


class SpotQueueInterface(ABC):
    """
    We need to queue different data for our spots.
    Said data is primarely defined by the joints and their timestamps.
    We assume that every spot, at any given time, has only one active user.
    Therefore, we assume that we can sort data from multiple views into the same spot-queue, providing their timestamps
    """
    @abstractmethod
    def dequeue(self, spot_key: str) -> Tuple[list, list, list]:
        """
        Dequeue spot data from queues and return it as a tuple:

        Returns a spot-info dictionary and three lists:
        * The first is a list of lists of joints, possibly with additional data, representing the past.
        * The second is a list of joints, possibly with additional data.
        * The third is a list of lists of joints, possibly with additional data, representing the future.

        Return spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list
        """
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def enqueue(self, spot_key: str, message: Any) -> None:
        """ 
        Enqueue a list of joints, possibly with additional data.
        """
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def size(self, spot_key: str):
        """ Returns the size of the Spot Queue. """
        raise NotImplementedError("This is an interface and shold not be called directly")

    def delete(self, spot_key: str):
        """ Deletes the content of a spot queue. """
        raise NotImplementedError("This is an interface and shold not be called directly")


class MessageQueueInterface(ABC):
    @abstractmethod
    def enqueue(self, spot_key: str, message: Any) -> None:
        """ 
        Enqueue a dictionary, which can be sent as a message somewhere else.
        """
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def dequeue(self, spot_key: str) -> dict:
        """ 
        Dequeue a dictionary, which can be sent as a message somewhere else.
        """
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def delete(self, spot_key: str) -> None:
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def size(self, spot_key) -> Tuple[int, int, int]:
        raise NotImplementedError("This is an interface and shold not be called directly")


class QueueLoadBalancerInterface(ABC):
    @abstractmethod
    def __init__(self):
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def get_queue_key(self) -> str:
        """ Get the key for the spot that needs our attention the most """
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def set_queue_size(self, key: str, queue_size: int) -> None:
        """ Set the size of a queue """
        raise NotImplementedError("This is an interface and shold not be called directly")


class SpotInfoInterface(ABC):
    @abstractmethod
    def __init__(self):
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def get_spot_info_dict(self, key: str):
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def set_spot_info_dict(self, key: str, spot_info_dict: dict):
        raise NotImplementedError("This is an interface and shold not be called directly")


class RedisSpotInfoInterface(SpotInfoInterface):
    def __init__(self):
        self.redis_connection = redis.StrictRedis(host='localhost', port=5678, db=0, charset="utf-8", decode_responses=True)
        # TODO: Stop using msgpack here and use something easier

    def get_spot_info_dict(self, key: str, info_keys: list):
        spot_info_list = self.redis_connection.hmget(key, info_keys)
        if not spot_info_list:
            raise QueueingException("Trying to process queue with key " + spot_info_key + " which has incomplete information set (maybe no exercise was set?)")
        spot_info_dict = dict(zip(info_keys, spot_info_list))
        if "exercise_data" in spot_info_dict.keys() and spot_info_dict["exercise_data"]:
            spot_info_dict["exercise_data"] = yaml.load(spot_info_dict["exercise_data"], Loader=yaml.Loader)
        return spot_info_dict

    def set_spot_info_dict(self, key: str, spot_info_dict: dict):
        if "exercise_data" in spot_info_dict.keys():
            yaml_string = yaml.dump(spot_info_dict["exercise_data"])
            spot_info_dict["exercise_data"] = yaml_string
            spot_info_dict["exercise_data_hash"] = hashlib.md5(yaml_string.encode('utf-8')).hexdigest()
        self.redis_connection.hset(key, mapping=spot_info_dict)


class RedisQueueLoadBalancerInterface(QueueLoadBalancerInterface):
    def __init__(self):
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)

    def get_queue_key(self):
        # For now, simply take the key corresponding to the spot with the highest load, NO ACTUALY LOAD BALANCING!
        spot_key_list = self.redis_connection.zrange(REDIS_LOAD_BALANCER_LIST_KEY, 0, 0)

        if not spot_key_list:
           raise QueueEmpty
        spot_key = spot_key_list[0]
        try:
            spot_key = spot_key.decode('utf-8')
        except Exception:
            # Catch cases where we can an byte encoded string back (ROS Melodic's Python 2.7 does that to us)
            pass
                
        new_score = self.redis_connection.zadd(REDIS_LOAD_BALANCER_LIST_KEY, {spot_key: -1}, incr=True)

        if new_score < 0:
            self.redis_connection.zrem(REDIS_LOAD_BALANCER_LIST_KEY, spot_key)

        return spot_key

    def set_queue_size(self, key: str, queue_size: int) -> None:
        """ Set the size of a queue """

        if (queue_size >= STATION_QUEUE_SIZE_MINIMUM):
            # The queue_size serves as our score
            self.redis_connection.zadd(REDIS_LOAD_BALANCER_LIST_KEY, {key: queue_size})
        else:
            self.redis_connection.zrem(REDIS_LOAD_BALANCER_LIST_KEY, key)
        

class RedisMessageQueueInterface(MessageQueueInterface):
    def __init__(self):
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)

    def enqueue(self, key, message):
        message = self.redis_connection.lpush(key, msgpack.packb(message))

    def blocking_dequeue(self, key, timeout=2):
        message = self.redis_connection.brpop(key, timeout)
        if not message:
            raise QueueEmpty
        message = message[1]
        message = msgpack.unpackb(message) # TODO: Python 2.7 bytestrings anywhere?
        try:
            message = message.decode('utf-8')
        except Exception:
            # Catch cases where we can an byte encoded string back (ROS's Python 2.7 does that to us)
            pass
        
        return message
            
    def dequeue(self, key: str):
        try:
            message = self.redis_connection.rpop(key, timeout)
            if not message:
                raise QueueEmpty
            message = msgpack.unpackb(message) # TODO: Python 2.7 bytestrings anywhere?
            try:
                message = message.decode('utf-8')
            except Exception:
                # Catch cases where we can an byte encoded string back (ROS's Python 2.7 does that to us)
                pass
        except QueueEmpty:
            raise QueueEmpty
        except Exception as e:
            if HIGH_VERBOSITY:
                rp.logerr("Issue getting message from Queue: " + str(key))
                print_exc()
            pass
        
        return message

    def delete(self, key: str) -> None:
        return self.redis_connection.delete(key)

    def size(self, key: str) -> Tuple[int, int, int]:
        return self.redis_connection.llen(key)


class RedisSpotQueueInterface(SpotQueueInterface):
    """
    This class helps with enqueue and dequeueing from and to a redis database enqueue and dequeue data that the comparator needs with Redis.

    Our Redis FIFO Queues that contain joints that are produced by whatever AI our System uses.

    * The 'queue' for a spot is expected to contain a list of of dictionaries '{'bodyParts': <Bodyparts>, 'time_stamp': <header.stamp>}'.
    These dictionaries represent data not yet processed by the comparator, or 'the future'.
    * The 'past queue' for a spot is expected to contains the dictionaries already processed, or 'the past'.
    * The info for a spot contains additional information like timing data in a dictionary.
    """

    def __init__(self):
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)
        # Dequeueing consists of several operations. Until we move to a distribuetd system, we can use a semaphore to guarantee correct queuing.
        # Redis has distributed locks: https://redis.io/topics/distlock

    def dequeue(self, spot_key: str) -> Tuple[list, list, list]:
        """
        Return spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list
        """
        # TODO: Investigate if these redis instructions can be optimized, possibly use a distributed lock

        redis_spot_queue_key, redis_spot_past_queue_key, redis_spot_info_key = generate_redis_key_names(spot_key)

        try:
            joints_with_timestame_bytes = self.redis_connection.rpoplpush(redis_spot_queue_key, redis_spot_past_queue_key)
            if not joints_with_timestame_bytes:
                raise QueueEmpty
            joints_with_timestamp = msgpack.unpackb(joints_with_timestame_bytes) 
            assert joints_with_timestamp
        except (KeyError, AssertionError):
            print_exc()
            # Supposingly, no message queue is holding any value (at the start of the system)
            raise QueueEmpty

        self.redis_connection.ltrim(redis_spot_past_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)

        future_joints_with_timestamp_list = self.redis_connection.lrange(redis_spot_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)
        past_joints_with_timestamp_list = self.redis_connection.lrange(redis_spot_past_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)

        return past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list

    def enqueue(self, spot_key: str, data: Any) -> int:
        """ Return queue size """
        redis_spot_queue_key, _, _ = generate_redis_key_names(spot_key)

        queue_size = self.redis_connection.rpush(redis_spot_queue_key, msgpack.packb(data))

        if (queue_size >= REDIS_MAXIMUM_QUEUE_SIZE):
            if HIGH_VERBOSITY:
                rp.logerr("Maximum Queue size for spot with key " + str(spot_key) + " reached. Removing first element.")
            self.redis_connection.ltrim(redis_spot_queue_key, 0, REDIS_MAXIMUM_QUEUE_SIZE)

        return queue_size

    def size(self, spot_key: str):
        redis_spot_queue_key, redis_spot_past_queue_key, redis_spot_info_key = generate_redis_key_names(key)
        return self.redis_connection.llen(redis_spot_queue_key), self.redis_connection.llen(redis_spot_past_queue_key), self.redis_connection.llen(redis_spot_info_key)

    def delete(self, spot_key: str):
        # TODO: Try and catch
        keys = generate_redis_key_names(key)
        for key in keys:
            self.redis_connection.delete(key)
            

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
    redis_spot_queue_key = REDIS_GENERAL_PREFIX + str(spot_key) + REDIS_SPOT_QUEUE_POSTFIX
    redis_spot_past_queue_key = REDIS_GENERAL_PREFIX + redis_spot_queue_key + REDIS_SPOT_PAST_QUEUE_POSTFIX
    redis_spot_info_key = REDIS_GENERAL_PREFIX + str(spot_key) + REDIS_SPOT_INFO_POSTFIX

    return redis_spot_queue_key, redis_spot_past_queue_key, redis_spot_info_key

