"""
This file contains a code snippets that have nowhere else to go.
"""

import sys
from functools import reduce
from typing import Any, Dict

import msgpack
import msgpack_numpy as m
import numpy as np
import rospy as rp
import json
import collections

try:
    from comparing_system.src.config import *
except ImportError:
    from src.config import *

m.patch()

# Alias for features states
FEATURE_LOW: int = -1
FEATURE_UNDECIDED: int = 0
FEATURE_HIGH: int = 1

# TODO: Check more sizes of exercise objects frequently
def get_size_of_object(obj, seen=None):
    """
    Recursively finds size of objects
    Taken from: https://goshippo.com/blog/measure-real-size-any-python-object/
    """
    size = sys.getsizeof(obj)
    if seen is None:
        seen = set()
    obj_id = id(obj)
    if obj_id in seen:
        return 0
    # Important mark as seen *before* entering recursion to gracefully handle
    # self-referential objects
    seen.add(obj_id)
    if isinstance(obj, dict):
        size += sum([get_size(v, seen) for v in obj.values()])
        size += sum([get_size(k, seen) for k in obj.keys()])
    elif hasattr(obj, '__dict__'):
        size += get_size(obj.__dict__, seen)
    elif hasattr(obj, '__iter__') and not isinstance(obj, (str, bytes, bytearray)):
        size += sum([get_size(i, seen) for i in obj])
    return size


def publish_message(publisher, topic, data):
    try:
        # This is currently how our REST API wants messages to be formed
        message = json.dumps({'topic': topic, 'data': data})
        publisher.publish(message)
        if HIGH_VERBOSITY:
            rp.logerr("ComparingSystem_Sender.py sent message: " + str(message))
    except Exception as e:
        if HIGH_VERBOSITY:
            rp.logerr("Issue sending message" + str(message) + " to REST API. Error: " + str(e))


def enqueue_dictionary(previous_dict, enqueued_dict):
    for k, v in enqueued_dict.items():
        if isinstance(v, collections.MutableMapping):
            previous_dict[k] = enqueue_dictionary(previous_dict.get(k, {}), v)
        elif isinstance(v, list):
            previous_dict[k] = previous_dict.get(k, [])
            previous_dict[k].extend(v)
            if (len(previous_dict[k]) >= REDIS_MAXIMUM_QUEUE_SIZE):
                previous_dict[k] = previous_dict[k][0: REDIS_MAXIMUM_QUEUE_SIZE]
        else:
            previous_dict[k] = previous_dict.get(k, [])
            if previous_dict[k] == None:
                previous_dict[k] = []
            previous_dict[k].append(v)
            if (len(previous_dict[k]) >= REDIS_MAXIMUM_QUEUE_SIZE):
                previous_dict[k] = previous_dict[k][0: REDIS_MAXIMUM_QUEUE_SIZE]
    
    return previous_dict


def compute_new_feature_progression(beginning_state, features_state, last_feature_progression):
    """Compute a dictionary representing the progression of the features specified by feature_state

    This method turns features states, such as FEATURE_HIGH or FEATURE_LOW into a feature progression,
    that is a number that represents the number of state changes in this feature in this repetition, 
    depending on the previous progression of the feature. 
    This way we can track the progression of different features between timesteps.

    Args: 
        beginning_state: A dictionary that holds the state in which a feature begins for every feature of every category
        features_state: The state that the featuers are in
        last_feature_progression: The last dictionary produced by this method in the last timestep

    Return:
        new_feature_progression: The feature progression dictionary at this timestep

    Raises:
        MalformedFeatures: If features are not the expected form.
    """
    new_feature_progression = last_feature_progression

    # If features beginn with the FEATURE_HIGH state, feature progressions must be odd if the feature state changes to FEATURE_LOW and even afterwards
    if beginning_state == FEATURE_HIGH and \
        ((features_state == FEATURE_LOW and last_feature_progression % 2 == 0) or \
            (features_state == FEATURE_HIGH and last_feature_progression % 2 == 1)):
            new_feature_progression = last_feature_progression + 1
    # If features beginn with the FEATURE_LOW state, feature progressions must be odd if the feature state changes to FEATURE_HIGH and even afterwards
    elif beginning_state == FEATURE_LOW and \
        ((features_state == FEATURE_HIGH and last_feature_progression % 2 == 0) or \
            (features_state == FEATURE_LOW and last_feature_progression % 2 == 1)):
            new_feature_progression = last_feature_progression + 1
    
    return new_feature_progression