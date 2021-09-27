#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Receiver Node.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

import rospy as rp
from typing import Any
 
try:
    from motion_analysis.src.Worker import *
    from motion_analysis.src.DataConfig import *
    from motion_analysis.src.InterCom import *
    from motion_analysis.src.DataUtils import *
    from motion_analysis.src.ROSAdapters import *
    from motion_analysis.src.algorithm.AlgoConfig import *
    from motion_analysis.src.algorithm.FeatureExtraction import *
    from motion_analysis.src.algorithm.AlgoUtils import *
    from backend.msg import Persons
except ImportError:
    from src.Worker import *
    from src.DataConfig import *
    from src.InterCom import *
    from src.DataUtils import *
    from src.ROSAdapters import *
    from src.algorithm.AlgoConfig import *
    from src.algorithm.FeatureExtraction import *
    from src.algorithm.AlgoUtils import *
    from backend.msg import Persons


class Receiver():
    """
    The Receiver subscribes to the ROS topic that contains joints (or 'skelletons') and puts them into separate queues.
    See the src/backend/msg/Persons Message for further information.
    Each queue corresponds to one spot. Therefore, multiple views of the same spot go into the same queue.
    """
    def __init__(self, 
    spot_queue_load_balancer_class: QueueLoadBalancerInterface = RedisQueueLoadBalancerInterface, 
    spot_queue_interface_class: SpotQueueInterface = RedisSpotQueueInterface,
    feature_extractor_class: PoseDefinitionAdapter = MetrabsPoseDefinitionAdapter):
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
    rp.init_node('Motion_Analysis_Receiver', anonymous=False)

    receiver = Receiver()

    rp.spin()

 