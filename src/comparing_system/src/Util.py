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

try:
    from comparing_system.src.config import *
except ImportError:
    from src.config import *

m.patch()


# Alias for features states
FEATURE_LOW: str = 'low'
FEATURE_UNDECIDED: str = 'undecided'
FEATURE_HIGH: str = 'high'

# Aliases for feature progressions
PROGRESSION_START: str = 'start'
PROGRESSION_PARTIAL: str = 'partial'
PROGRESSION_DONE: str = 'done'


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
