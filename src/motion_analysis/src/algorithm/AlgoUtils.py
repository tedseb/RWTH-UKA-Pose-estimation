#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains a code snippets that have nowhere else to go.
"""

import sys
from functools import reduce
from typing import Any, Dict

import msgpack_numpy as m
import numpy as np
import json
import collections

try:
    from motion_analysis.src.algorithm.AlgoConfig import *
except (ModuleNotFoundError, ImportError):
    from src.algorithm.AlgoConfig import *

m.patch()

class ContinueToOuterLoop(Exception):
    pass

# Alias for features states
FEATURE_LOW: int = -2
FEATURE_LOW_UNDECIDED = -1
FEATURE_UNDECIDED: int = 0
FEATURE_HIGH_UNDECIDED = 1
FEATURE_HIGH: int = 2

def try_import_rp():
    if "rospy" in sys.modules:
        rp_module = sys.modules["rospy"]
    else:
        try:
            import comparing_system.src.FakeRospy as FakeRospy
        except ImportError:
            import src.FakeRospy as FakeRospy
        
        rp_module = FakeRospy

    return rp_module

rp = try_import_rp()
    
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


def enqueue_dictionary(previous_dict, enqueued_dict):
    for k, v in enqueued_dict.items():
        if isinstance(v, collections.MutableMapping):
            previous_dict[k] = enqueue_dictionary(previous_dict.get(k, {}), v)
        elif isinstance(v, list):
            previous_dict[k] = previous_dict.get(k, [])
            previous_dict[k].extend(v)
            if (len(previous_dict[k]) >= REDIS_MAXIMUM_QUEUE_SIZE):
                previous_dict[k] = previous_dict[k][-REDIS_MAXIMUM_QUEUE_SIZE:]
        else:
            previous_dict[k] = previous_dict.get(k, [])
            if previous_dict[k] == None:
                previous_dict[k] = []
            previous_dict[k].append(v)
            if (len(previous_dict[k]) >= REDIS_MAXIMUM_QUEUE_SIZE):
                previous_dict[k] = previous_dict[k][-REDIS_MAXIMUM_QUEUE_SIZE:]
    
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
    # If features beginn with the FEATURE_HIGH state, feature progressions must be odd if the feature state changes to FEATURE_LOW and even afterwards
    if beginning_state == FEATURE_HIGH and \
        ((features_state <= FEATURE_LOW_UNDECIDED and last_feature_progression % 2 == 0) or \
            (features_state >= FEATURE_HIGH_UNDECIDED and last_feature_progression % 2 == 1)):
            new_feature_progression = last_feature_progression + 1
    # If features beginn with the FEATURE_LOW state, feature progressions must be odd if the feature state changes to FEATURE_HIGH and even afterwards
    elif beginning_state == FEATURE_LOW and \
        ((features_state >= FEATURE_HIGH_UNDECIDED and last_feature_progression % 2 == 0) or \
            (features_state <= FEATURE_LOW_UNDECIDED and last_feature_progression % 2 == 1)):
            new_feature_progression = last_feature_progression + 1
    else:
        new_feature_progression = last_feature_progression

    return new_feature_progression


def remove_jitter_from_trajectory(trajectory, _range):
    """Remove repeated ups and downs in a trajectory.
    
    A trajectory may be subjected to quick up and down movement as unreliable measurements are taken.
    For example measurements of an angle can jitter if the adjecent joints jitter.
    With this method we remove such measurements from a trajectory.
    
    Args:
        trajectory: A list of values that we want to free from jitter.
        _range: The range of values that can be part of the jitter. i.e. 3 ups and 3 downs is a range of 3.
        
    Return:
        The trimmed trajectory
    """
    done = False
    while not done:
        done = True
        try:
            for i in range(1, _range):
                for j in range(i, len(trajectory) - 3 * i):
                    window = trajectory[j: j + 3 * i]
                    if np.array_equal(window[:i], window[2 * i:]):
                        done = False
                        try:
                            del(trajectory[j: j + 3 * i])
                        except ValueError:
                            trajectory = np.delete(trajectory, range(j, j + 3 * i - 1))
                            raise ContinueToOuterLoop
        except ContinueToOuterLoop:
            pass
                        
    return trajectory


