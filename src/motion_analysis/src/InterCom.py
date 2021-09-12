#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
In order to implement low latency, high throughput and scalable queueing, we may want to employ different queues.
This file defines the several interfaces that cover queueing of data and handling of shared data.
It also offers implementations of said interfaces. 
The queueing should be trasparent to the Comparator and the ComparingNode.
"""

import hashlib
from abc import ABC, abstractmethod
from traceback import print_exc
from typing import Any, Dict, List, Tuple
from copy import deepcopy

import collections
import msgpack  # TODO: Use Protobufs?
import msgpack_numpy as m
import redis
import yaml
import rospy as rp
import pickle

try:
    from motion_analysis.src.DataConfig import *
except ImportError:
    from src.DataConfig import *

# Patch msgpack to understand numpy arrays
m.patch()


redis_connection_pool = redis.ConnectionPool(host='localhost', port=5678, db=0)


class QueueingException(Exception):
    pass


class QueueEmpty(QueueingException):
    pass


class QueueTimeout(QueueingException):
    pass


class SpotMetaDataException(Exception):
    pass


class NoSpotMetaDataSetExcpetion(SpotMetaDataException):
    pass


class SpotQueueInterface(ABC):
    """This interface queue pose data and timestamps for our spots.

    We assume that every spot, at any given time, has only one active user.
    Therefore, we assume that we can sort data from multiple views into the same spot-queue, provided they appear quasi-instantanious
    """
    @abstractmethod
    def dequeue(self, spot_key: str) -> Tuple[list, list, list]:
        """Dequeue spot data from queues and return it as a tuple:

        Returns:
            Three lists: 
            * The first list is a list of lists of joints, possibly with additional data, representing the past.
            * The second is a list of joints, possibly with additional data.
            * The third is a list of lists of joints, possibly with additional data, representing the future.

        Return:
            spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list
        """
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def enqueue(self, spot_key: str, message: Any) -> None:
        """Enqueue a list of joints, possibly with additional data."""
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def size(self, spot_key: str):
        """Return the size of the Spot Queue. """
        raise NotImplementedError("This is an interface and shold not be called directly")

    def delete(self, spot_key: str):
        """Delete the content of a spot queue. """
        raise NotImplementedError("This is an interface and shold not be called directly")


class MessageQueueInterface(ABC):
    """This interface provides access to message queues for outgoing messages concerning singular spots."""
    @abstractmethod
    def enqueue(self, spot_key: str, message: Any) -> None:
        """Enqueue a dictionary, which can be sent as a message somewhere else."""
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def dequeue(self, spot_key: str) -> dict:
        """Dequeue a dictionary, which can be sent as a message somewhere else."""
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def delete(self, spot_key: str) -> None:
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def size(self, spot_key) -> int:
        raise NotImplementedError("This is an interface and shold not be called directly")


class QueueLoadBalancerInterface(ABC):
    """This load balancing interface provides implements what queue we should dequeue from."""
    @abstractmethod
    def __init__(self):
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def get_queue_key(self) -> str:
        """Get the key for the spot that needs our attention the most."""
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def set_queue_size(self, key: str, queue_size: int) -> None:
        """Set the size of a queue """
        raise NotImplementedError("This is an interface and shold not be called directly")


class SpotMetaDataInterface(ABC):
    """This interface handles metadata about spots, namely statistics, the used exercise, and others."""
    @abstractmethod
    def __init__(self):
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def get_spot_info_dict(self, key: str, info_keys: list) -> dict:
        """Get the information related to a spot, providing the information keys."""
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def set_spot_info_dict(self, key: str, spot_info_dict: dict) -> None:
        """Set the information related to a spot as a dictionary."""
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def delete(self, key: str) -> None:
        raise NotImplementedError("This is an interface and shold not be called directly")


class FeaturesInterface(ABC):
    """This interface queues past features of interest for a spot.

    Said features have separate queues with keys that are generated on the way, using featires_of_interest_specification dictionaries.    
    """
    @abstractmethod
    def get(self, spot_key: str, latest_only: bool = False) -> List[dict]:
        """Get the feature queues of a spot, according to the features_of_interest_specification."""
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def set(self, spot_key: str, featuers_dict: dict,  reset: bool = False) -> None:
        """Put the features of a spot, according to the features_of_interest_specification."""
        raise NotImplementedError("This is an interface and shold not be called directly")

    def delete(self, spot_key: str) -> None:
        """Delete the content of a spot queue."""
        raise NotImplementedError("This is an interface and shold not be called directly")


class RedisInterface:
    def __init__(self):
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)

    def delete(self, key: str) -> None:
        return self.redis_connection.delete(key)


class RedisFeaturesInterface(RedisInterface, FeaturesInterface):
    # TODO: These methods will let the Redis memory grow overtime, as different exercises are used. Prevent this.
    # TODO: Use redis pipelines
    # TODO: Use hmap or scan+get, test different implementations
    def __init__(self):
        # We need a separate constructor in order to force decode_responses=True for this Redis connection
        self.redis_connection = redis.StrictRedis(host='localhost', port=5678, db=0, decode_responses=False)

    def get(self, spot_features_key: str) -> dict:
        # TODO: Optimize this
        features_dict = self.redis_connection.hgetall(spot_features_key)
        for key, feature_string in features_dict.items():
            features_dict[key] = pickle.loads(feature_string)
        return features_dict

    def set(self, spot_features_key: str, features_dict: dict) -> None:
        _features_dict = deepcopy(features_dict)
        for key, feature in _features_dict.items():
            _features_dict[key] = pickle.dumps(feature)
                
        self.redis_connection.hset(spot_features_key, mapping=_features_dict)

    def delete(self, spot_features_key: str) -> None:
        self.redis_connection.delete(spot_features_key)
        

class RedisSpotMetaDataInterface(RedisInterface, SpotMetaDataInterface):
    def __init__(self):
        # TODO: Use Messagepack here
        # We need a separate constructor in order to force decode_responses=True for this Redis connection
        self.redis_connection = redis.StrictRedis(host='localhost', port=5678, db=0, decode_responses=True)

    def get_spot_info_dict(self, key: str, info_keys: list) -> dict:
        spot_info_list = self.redis_connection.hmget(key, info_keys)
        if None in spot_info_list:
            raise NoSpotMetaDataSetExcpetion("Trying to get spot_info_dict at key " + str(key) + ", but no spot_info_dict is set.")
        spot_info_dict = dict(zip(info_keys, spot_info_list))
        if "exercise_data" in spot_info_dict.keys() and spot_info_dict["exercise_data"]:
            spot_info_dict["exercise_data"] = yaml.load(spot_info_dict["exercise_data"], Loader=yaml.Loader)
        return spot_info_dict

    def set_spot_info_dict(self, key: str, spot_info_dict: dict) -> None:
        spot_info_dict_copy = deepcopy(spot_info_dict)
        if "exercise_data" in spot_info_dict.keys():
            yaml_string = yaml.dump(spot_info_dict_copy["exercise_data"])
            spot_info_dict_copy["exercise_data"] = yaml_string
            spot_info_dict_copy["exercise_data_hash"] = hashlib.md5(yaml_string.encode('utf-8')).hexdigest()
        self.redis_connection.hset(key, mapping=spot_info_dict_copy)

    def delete(self, key: str) -> None:
        self.redis_connection.delete(key)


class RedisQueueLoadBalancerInterface(RedisInterface, QueueLoadBalancerInterface):
    def get_queue_key(self):
        # For now, we want a thread to always focus on the largest queue
        try:
            key_tuple = self.redis_connection.bzpopmax(REDIS_LOAD_BALANCER_SORTED_SET_KEY, timeout=1)
        except TimeoutError:
            raise QueueEmpty
        
        if key_tuple is None:
            raise QueueEmpty
        
        # In the future, we can use expire commands a track working comparators for better balancing
        # self.redis_connection.incrby(REDIS_LOAD_BALANCER_ACTIVE_COMPARATORS_SORTED_SET_KEY + spot_key, amount=1)
        # self.redis_connection.pexpire(REDIS_LOAD_BALANCER_ACTIVE_COMPARATORS_SORTED_SET_KEY + spot_key, 100)

        # Set score to zero because we now
        spot_key = key_tuple[1]
        try:
            spot_key = spot_key.decode('utf-8')
        except Exception:
            # Catch cases where we can an byte encoded string back (ROS Melodic's Python 2.7 does that to us)
            pass
        return spot_key

    def set_queue_size(self, key: str, size: int = 1) -> None:
        """Increment the size of a queue as seen by the load balancer."""
        self.redis_connection.zadd(REDIS_LOAD_BALANCER_SORTED_SET_KEY, {key: size})
    

class RedisMessageQueueInterface(RedisInterface, MessageQueueInterface):
    def enqueue(self, key, message) -> None:
        message = self.redis_connection.lpush(key, msgpack.packb(message))

    def blocking_dequeue(self, key, timeout=2) -> object:
        message = self.redis_connection.brpop(key, timeout)
        if not message:
            raise QueueEmpty
        message = message[1]
        message = msgpack.unpackb(message)
        try:
            message = message.decode('utf-8')
        except Exception:
            # Catch cases where we can an byte encoded string back (ROS's Python 2.7 does that to us)
            pass
        
        return message
            
    def dequeue(self, key: str) -> object:
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
        
        return message

    def size(self, key: str) -> Tuple[int, int, int]:
        return self.redis_connection.llen(key)


class RedisSpotQueueInterface(RedisInterface, SpotQueueInterface):
    def __init__(self):
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)
        # Dequeueing consists of several operations. Until we move to a distribuetd system, we can use a semaphore to guarantee correct queuing.
        # Redis has distributed locks: https://redis.io/topics/distlock

    def dequeue(self, spot_key: str) -> Tuple[list, list, list]:
        # TODO: Maybe use https://github.com/RedisTimeSeries/RedisTimeSeries
        # TODO: Investigate if these redis instructions can be optimized, possibly use a distributed lock

        spot_queue_key, spot_past_queue_key, _, _ = generate_redis_key_names(spot_key)

        try:
            joints_with_timestame_bytes = self.redis_connection.rpoplpush(spot_queue_key, spot_past_queue_key)
            if not joints_with_timestame_bytes:
                raise QueueEmpty
            joints_with_timestamp = msgpack.unpackb(joints_with_timestame_bytes) 
            assert joints_with_timestamp
        except (KeyError, AssertionError):
            print_exc()
            # Supposingly, no message queue is holding any value (at the start of the system)
            raise QueueEmpty

        self.redis_connection.ltrim(spot_past_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)

        future_joints_with_timestamp_list = self.redis_connection.lrange(spot_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)
        past_joints_with_timestamp_list = self.redis_connection.lrange(spot_past_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)

        return past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list

    def enqueue(self, spot_key: str, data: Any) -> int:
        spot_queue_key, _, _, _ = generate_redis_key_names(spot_key)

        queue_size = self.redis_connection.rpush(spot_queue_key, msgpack.packb(data))

        if (queue_size >= REDIS_QUEUE_SIZE_PANIC_BOUNDARY):
            if (queue_size >= REDIS_MAXIMUM_QUEUE_SIZE):
                if HIGH_VERBOSITY:
                    rp.logerr("Maximum Queue size for spot with key " + str(spot_key) + " reached. Removing first element.")
                self.redis_connection.ltrim(spot_queue_key, 0, REDIS_MAXIMUM_QUEUE_SIZE)
            else:
                if HIGH_VERBOSITY:
                    return 1
                    rp.logerr("Queue panic boundary for queue with key " + str(spot_key) + " reached. Increasing load balancing priority...")
        return 1

    def size(self, spot_key: str):
        spot_queue_key, spot_past_queue_key, spot_info_key, spot_features_key  = generate_redis_key_names(spot_key)
        return self.redis_connection.llen(spot_queue_key), self.redis_connection.llen(spot_past_queue_key), self.redis_connection.llen(spot_info_key), self.redis_connection.llen(spot_features_key)

    def delete(self, spot_key: str):
        spot_queue_key, spot_past_queue_key, _,  _ = generate_redis_key_names(spot_key)
        self.redis_connection.delete(spot_queue_key)
        self.redis_connection.delete(spot_past_queue_key)
            

def generate_redis_key_names(spot_key: str) -> List[str]:
    """This method simplifies handling of general key names.

    Returns: 
        redis_spot_queue_key, redis_spot_past_queue_key, redis_spot_info_key, redis_spot_state_key

    Data that is expected by the comparator is listed below.
    The separator ':' is used here, allthough this can be changed with REDIS_KEY_SEPARATOR.

    The redis keys, structures of their values and types are as follows:
        * Spot with ID N is:                    <spot_key>                          :(prefix for other data)
        * Queue for spot with ID N is           <spot_key:queue>                    :Redis FIFO Queue
        * Past queue for spot with ID N is      <spot_key:queue_past>               :Redis FIFO Queue
        * Info for spot with ID N is            <spot_key:info>                     :Redis Hashmap
        * Features of spot with ID N is         <spot_key:features:*>               :multiple Redis FIFO Queues
    """
    redis_spot_queue_key = REDIS_GENERAL_PREFIX + REDIS_KEY_SEPARATOR + str(spot_key) + REDIS_KEY_SEPARATOR + REDIS_SPOT_QUEUE_POSTFIX
    redis_spot_past_queue_key = REDIS_GENERAL_PREFIX + REDIS_KEY_SEPARATOR + redis_spot_queue_key + REDIS_KEY_SEPARATOR + REDIS_SPOT_PAST_QUEUE_POSTFIX
    redis_spot_info_key = REDIS_GENERAL_PREFIX + REDIS_KEY_SEPARATOR + str(spot_key) + REDIS_KEY_SEPARATOR + REDIS_SPOT_INFO_POSTFIX
    redis_spot_features_key = REDIS_GENERAL_PREFIX + REDIS_KEY_SEPARATOR + str(spot_key) + REDIS_KEY_SEPARATOR + REDIS_SPOT_FEATURES

    return redis_spot_queue_key, redis_spot_past_queue_key, redis_spot_info_key, redis_spot_features_key

