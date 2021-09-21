#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains code that abstracts logging from ROS and gives us the possibility to run tests without running ros"
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

from abc import abstractmethod

# Comparing system talks lots if this is set to true
HIGH_VERBOSITY = True

try:
    import rospy as rp
    if HIGH_VERBOSITY:
        def log(message):
            rp.loginfo(message)
        def log_throttle(message):
            rp.loginfo_throttle(5, message)
    else:
        def log(message):
            rp.logerr(message)
        def log_throttle(message):
            rp.logerr_throttle(5, message)
except Exception as e:
    print(e)
    print("Could not load rospy logger, resorting to prints...")
    def log(message):
        print(message)
    def log_throttle(message):
        print("(unthrottled log) " + str(message))
