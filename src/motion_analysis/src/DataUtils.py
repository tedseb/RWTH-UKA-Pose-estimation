#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains a code snippets that have nowhere else to go.
"""

import json
import rospy as rp

try:
    from motion_analysis.src.algorithm.logging import log
except ImportError:
    from src.algorithm.logging import log

def publish_message(publisher, topic, data):
    try:
        # This is currently how our REST API wants messages to be formed
        message = json.dumps({'topic': topic, 'data': data})
        publisher.publish(message)
    except Exception as e:
        rp.logerr(e)
        log("Issue sending message" + str(message) + " to REST API. Error: " + str(e))
