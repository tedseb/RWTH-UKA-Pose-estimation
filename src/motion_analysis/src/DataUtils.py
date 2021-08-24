#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains a code snippets that have nowhere else to go.
"""

import json
import rospy as rp

try:
    from motion_analysis.src.DataConfig import *
except ImportError:
    from src.DataConfig import *

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
