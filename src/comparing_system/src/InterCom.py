"""
In order to implement low latency, high throughput and scalable queueing, we may want to employ different queues.
This file defines the several interfaces that cover queueing of data and handling of shared data.
It also offers implementations of said interfaces. 
The queueing should be trasparent to the Comparator and the ComparingNode.
"""

import hashlib
import threading
from abc import ABC, abstractmethod
from collections import OrderedDict
from traceback import print_exc
from typing import Any, Dict, List, Tuple

import msgpack  # TODO: Use Protobufs
import msgpack_numpy as m
import redis
import rospy as rp
import yaml
from src.config import *

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
    """This interface queue different data for our spots.

    Said data consists of arrays of joints and their timestamps.
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

        Return spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list
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
    def get_spot_state_dict(self, key: str, feature_of_interest_specification: dict) -> dict:
        """Get the state related to a spot, provoding a specification dictionary of featuers of interest."""
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def set_spot_state_dict(self, key: str, spot_info_dict: dict) -> None:
        """Set the state of a spot as a dictionary."""
        raise NotImplementedError("This is an interface and shold not be called directly")

    @abstractmethod
    def delete(self, key: str) -> None:
        raise NotImplementedError("This is an interface and shold not be called directly")


class RedisInterface:
    def __init__(self):
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)

    def delete(self, key: str) -> None:
        return self.redis_connection.delete(key)
        

class RedisSpotMetaDataInterface(RedisInterface, SpotMetaDataInterface):
    def __init__(self):
        # TODO: Use Messagepack here
        # We need a separate constructor in order to force decode_responses=True for this Redis connection
        self.redis_connection = redis.StrictRedis(host='localhost', port=5678, db=0, decode_responses=True)

    def get_spot_info_dict(self, key: str, info_keys: list) -> dict:
        spot_info_list = self.redis_connection.hmget(key, info_keys)
        if None in spot_info_list:
            raise NoSpotMetaDataSetExcpetion("Trying to get spot_info_dict at key " + key + ", but no spot_info_dict is set.")
        spot_info_dict = dict(zip(info_keys, spot_info_list))
        if "exercise_data" in spot_info_dict.keys() and spot_info_dict["exercise_data"]:
            spot_info_dict["exercise_data"] = yaml.load(spot_info_dict["exercise_data"], Loader=yaml.Loader)
        return spot_info_dict

    def get_spot_state_dict(self, key: str, feature_of_interest_specification: dict) -> dict:
        redis_distances_key, redis_angles_key, redis_rotation_key, redis_speed_key, redis_rotation_key = generate_redis_spot_state_keys(key)
        state_dict = dict()

        for feature_type, feature_state_dict in feature_of_interest_specification.items():
            # TODO: Incorporate other feature types here
            # TODO: Use pipelining of Redis queries
            if feature_type == 'angles':
                feature_state_dict = self.redis_connection.hgetall(redis_angles_key)
            else:
                raise NotImplementedError("Trying to extract boundaries for an unspecified feature type")

            if not feature_state_dict:
                raise SpotMetaDataException("Trying to get state from spot with key " + key + " which has incomplete information set (maybe no exercise was set?)")

            state_dict[feature_type] = feature_state_dict

        return state_dict

    def set_spot_info_dict(self, key: str, spot_info_dict: dict) -> None:
        if "exercise_data" in spot_info_dict.keys():
            yaml_string = yaml.dump(spot_info_dict["exercise_data"])
            spot_info_dict["exercise_data"] = yaml_string
            spot_info_dict["exercise_data_hash"] = hashlib.md5(yaml_string.encode('utf-8')).hexdigest()
        self.redis_connection.hset(key, mapping=spot_info_dict)

    def set_spot_state_dict(self, key: str, spot_state_dict: dict) -> None:
        redis_distances_key, redis_angles_key, redis_rotation_key, redis_speed_key, redis_rotation_key = generate_redis_spot_state_keys(key)
        
        for feature_type, feature_state_dict in spot_state_dict.items():
            if feature_type == 'angles':
                self.redis_connection.hset(redis_angles_key, mapping=feature_state_dict)
            else:
                raise NotImplementedError("Trying to extract boundaries for an unspecified feature type")
    
    def delete(self, key: str) -> None:
        keys = generate_redis_spot_state_keys(key)
        self.redis_connection.delete(key)
        for k in keys:
            self.redis_connection.delete(k)
        

class RedisQueueLoadBalancerInterface(RedisInterface, QueueLoadBalancerInterface):
    def get_queue_key(self):
        # For now, simply take the key corresponding to the spot with the highest load, NO PROPER LOAD BALANCING!
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
        """Set the size of a queue."""

        if (queue_size >= STATION_QUEUE_SIZE_MINIMUM):
            # The queue_size serves as our score
            self.redis_connection.zadd(REDIS_LOAD_BALANCER_LIST_KEY, {key: queue_size})
        else:
            self.redis_connection.zrem(REDIS_LOAD_BALANCER_LIST_KEY, key)
        

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
            pass
        
        return message

    def size(self, key: str) -> Tuple[int, int, int]:
        return self.redis_connection.llen(key)


class RedisSpotQueueInterface(RedisInterface, SpotQueueInterface):
    def __init__(self):
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)
        # Dequeueing consists of several operations. Until we move to a distribuetd system, we can use a semaphore to guarantee correct queuing.
        # Redis has distributed locks: https://redis.io/topics/distlock

    def dequeue(self, spot_key: str) -> Tuple[list, list, list]:
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

        if (queue_size >= REDIS_MAXIMUM_QUEUE_SIZE):
            if HIGH_VERBOSITY:
                rp.logerr("Maximum Queue size for spot with key " + str(spot_key) + " reached. Removing first element.")
            self.redis_connection.ltrim(spot_queue_key, 0, REDIS_MAXIMUM_QUEUE_SIZE)

        return queue_size

    def size(self, spot_key: str):
        spot_queue_key, spot_past_queue_key, spot_info_key, spot_state_key = generate_redis_key_names(key)
        return self.redis_connection.llen(spot_queue_key), self.redis_connection.llen(spot_past_queue_key), self.redis_connection.llen(spot_info_key), self.redis_connection.llen(spot_state_key)

    def delete(self, spot_key: str):
        # TODO: Try and catch
        keys = generate_redis_key_names(key)
        for key in keys:
            self.redis_connection.delete(key)
            

def generate_redis_key_names(spot_key: str) -> List[str]:
    """This method simplifies handling of general key names.

    Returns: 
        redis_spot_queue_key, redis_spot_past_queue_key, redis_spot_info_key, redis_spot_state_key

    Data that is expected by the comparator is listed below.

    The redis keys and values are as follows:
        * Spot with ID N is:                    <spot_key>               :(prefix for other data)
        * Queue for spot with ID N is           <spot_key:queue>         :Redis FIFO Queue
        * Past queue for spot with ID N is      <spot_key:queue_past>    :Redis FIFO Queue
        * Info for spot with ID N is            <spot_key:info>          :Redis Hashmap
        * Exercise-state of spot with ID N is   <spot_key:state>         :Redis Hashmap
    """
    redis_spot_queue_key = REDIS_GENERAL_PREFIX + str(spot_key) + REDIS_SPOT_QUEUE_POSTFIX
    redis_spot_past_queue_key = REDIS_GENERAL_PREFIX + redis_spot_queue_key + REDIS_SPOT_PAST_QUEUE_POSTFIX
    redis_spot_info_key = REDIS_GENERAL_PREFIX + str(spot_key) + REDIS_SPOT_INFO_POSTFIX
    redis_spot_state_key = REDIS_GENERAL_PREFIX + str(spot_key) + REDIS_SPOT_STATE_POSTFIX

    return redis_spot_queue_key, redis_spot_past_queue_key, redis_spot_info_key, redis_spot_state_key

def generate_redis_spot_state_keys(redis_spot_state_key: str) -> List[str]:
    """This method simplifies handling of redis spot state keys names.

    Returns: 
        redis_distances_key, redis_angles_key, redis_rotation_key, redis_speed_key, redis_rotation_key

    Data that is expected by the comparator is listed below.

    The redis keys and values are as follows:
        * Distances is      <spot_key:state:distances>      :Redis Hashmap
        * Angles is         <spot_key:state:angles>         :Redis Hashmap
        * Rotations is      <spot_key:state:rotations>      :Redis Hashmap
        * Speed is          <spot_key:state:speeds>         :Redis Hashmap
        * Accaleration is   <spot_key:state:accalerations>  :Redis Hashmap
    """
    redis_distances_key = redis_spot_state_key + ":distances"
    redis_angles_key = redis_spot_state_key + ":angles"
    redis_rotation_key = redis_spot_state_key + ":rotations"
    redis_speed_key = redis_spot_state_key + ":speeds"
    redis_rotation_key = redis_spot_state_key + ":accalerations"

    return redis_distances_key, redis_angles_key, redis_rotation_key, redis_speed_key, redis_rotation_key
