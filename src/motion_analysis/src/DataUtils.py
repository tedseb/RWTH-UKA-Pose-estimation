#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains a code snippets that have nowhere else to go.
"""

import json
import logy


def publish_message(publisher, topic, data):
    """Publish a message on ROS using a given publisher according to Orhan's preferences.

    Orhans likes messages to have the form {'topic': topic, 'data': data}, where topic is the ROS topic.
    """
    try:
        # This is currently how our REST API wants messages to be formed
        message = json.dumps({'topic': topic, 'data': data})
        publisher.publish(message)
    except Exception as e:
        logy.error_throttle("Issue sending message" +
                            str(message) + " to REST API. Error: " + str(e))
