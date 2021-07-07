#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Receiver Node.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

from typing import Any

import rospy as rp    
from backend.msg import Persons

try:
    from comparing_system.src.Comparator import Comparator
    from comparing_system.src.config import *
    from comparing_system.src.FeatureExtraction import *
    from comparing_system.src.InterCom import *
    from comparing_system.src.Util import *
except ImportError:
    from src.Comparator import Comparator
    from src.config import *
    from src.FeatureExtraction import *
    from src.InterCom import *
    from src.Util import *


class Receiver():
    """
    The Receiver subscribes to the ROS topic that contains joints (or 'skelletons') and puts them into separate queues.
    See the src/backend/msg/Persons Message for further information.
    Each queue corresponds to one spot. Therefore, multiple views of the same spot go into the same queue.
    """
    def __init__(self, 
    spot_queue_load_balancer_class: QueueLoadBalancerInterface = RedisQueueLoadBalancerInterface, 
    spot_queue_interface_class: SpotQueueInterface = RedisSpotQueueInterface,
    feature_extractor_class: PoseDefinitionAdapter = SpinPoseDefinitionAdapter):
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
            queue_size_increment_value = self.spot_queue_interface.enqueue(p.stationID, joints_with_timestamp)
            self.spot_queue_load_balancer.set_queue_size(p.stationID, queue_size_increment_value)

if __name__ == '__main__':
    # initialize ros node
    rp.init_node('ComparingSystem_Receiver', anonymous=False)

    receiver = Receiver()

    rp.spin()

 