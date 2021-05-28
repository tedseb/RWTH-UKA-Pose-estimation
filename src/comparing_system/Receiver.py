#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Receiver Node.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

from typing import Any

import rospy as rp
from src.FeatureExtraction import *
from src.config import *
from src.InterCom import *
from src.Util import *

from backend.msg import Persons

class Receiver():
    """
    The Receiver subscribes to the ROS topic that contains joints (or 'skelletons') and puts them into separate queues.
    See the src/backend/msg/Persons Message for further information.
    Each queue corresponds to one spot. Therefore, multiple views of the same spot go into the same queue.
    """
    def __init__(self, 
    spot_queue_load_balancer_class: type(QueueLoadBalancerInterface) = RedisQueueLoadBalancerInterface, 
    spot_queue_interface_class: type(SpotQueueInterface) = RedisSpotQueueInterface,
    feature_extractor_class: type(FeatureExtractor) = SpinFeatureExtractor):
        # Define a subscriber to retrive tracked bodies
        rp.Subscriber(ROS_JOINTS_TOPIC, Persons, self.callback)
        self.spot_queue_interface = spot_queue_interface_class()

        self.spot_queue_load_balancer = spot_queue_load_balancer_class()
        self.feature_extractor = feature_extractor_class()

    def callback(self, message: Any) -> None:
        """
        This function is called everytime a message is received by this node.
        It puts the arriving skelletons in the queues for their respective spots, such that they can be dequeued by the comparator thread.
        """
        # For every person in the image, sort their data into the correction spot queue in redis
        for p in message.persons:
            array = self.feature_extractor.body_parts_to_ndarray(p.bodyParts)
            joints_with_timestamp = {'used_joint_ndarray': array, 'ros_timestamp': message.header.stamp.to_time()}
            # This mostly returns 1. Only, if the queue is full, it will return the size of the queue
            queue_size_increment_value = self.spot_queue_interface.enqueue(p.stationID, joints_with_timestamp)
            self.spot_queue_load_balancer.increment_queue_size(p.stationID, queue_size_increment_value)

if __name__ == '__main__':
    # initialize ros node
    rp.init_node('ComparingSystem_Receiver', anonymous=False)

    receiver = Receiver()

    rp.spin()

 