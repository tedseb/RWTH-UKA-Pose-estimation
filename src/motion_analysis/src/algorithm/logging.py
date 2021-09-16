#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains code that abstracts logging from ROS and gives us the possibility to run tests without running ros"
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

from abc import abstractmethod
import rospy as rp


rospy_not_found = False



class Logger():
    @abstractmethod
    def log(self, message):
        pass

    @abstractmethod
    def logerr(self, message):
        pass

class RospyErrorLogger(Logger):
    def log(self, message):
        rp.log(message)

    def logerr(self, message):
        rp.logerr(message)


class RospyLogger(Logger):
    pass