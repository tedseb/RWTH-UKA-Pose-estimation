#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Receiver Node.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

import rospy as rp
from typing import Any
import argparse
from backend.msg import Persons

try:
    from motion_analysis.src.Worker import *
    from motion_analysis.src.InterCom import *
    from motion_analysis.src.DataUtils import *
    from motion_analysis.src.ROSAdapters import *
    from motion_analysis.src.algorithm.FeatureExtraction import *
    from motion_analysis.src.algorithm.AlgoUtils import *
except ImportError:
    from src.Worker import *
    from src.InterCom import *
    from src.DataUtils import *
    from src.ROSAdapters import *
    from src.algorithm.FeatureExtraction import *
    from src.algorithm.AlgoUtils import *


class Receiver():
    """
    The Receiver subscribes to the ROS topic that contains joints (or 'skelletons') and puts them into separate queues.
    See the src/backend/msg/Persons Message for further information.
    Each queue corresponds to one spot. Therefore, multiple views of the same spot go into the same queue.
    """
    def __init__(self, 
    config,
    spot_queue_interface_class: SpotQueueInterface = RedisSpotQueueInterface,
    pose_definition_adapter_class: PoseDefinitionAdapter = MetrabsPoseDefinitionAdapter):
        self.config = config
        rp.Subscriber(config["ROS_JOINTS_TOPIC"], Persons, self.callback)
        self.spot_queue_interface = spot_queue_interface_class(self.config)

        self.pose_definition_adapter = pose_definition_adapter_class()

    def callback(self, message: Any) -> None:
        """
        This function is called everytime a message is received by this node.
        It puts the arriving skelletons in the queues for their respective spots, such that they can be dequeued by the comparator thread.
        """
        # For every person in the image, sort their data into the correction spot queue in redis
        for p in message.persons:
            array = self.pose_definition_adapter.body_parts_to_ndarray(p.bodyParts)
            joints_with_timestamp = {'used_joint_ndarray': array, 'ros_timestamp': message.header.stamp.to_time()}
            queue_size_increment_value = self.spot_queue_interface.enqueue(p.stationID, joints_with_timestamp)

if __name__ == '__main__':
    # initialize ros node
    rp.init_node('Motion_Analysis_Receiver', anonymous=False)

    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="Verbose mode", action="store_true") # TODO: @sm This does not do anything now - is it even necessary?
    parser.add_argument("-a", "--ai", help="Name of AI to work with", type=str, default='metrabs')
    parser.add_argument("-c", "--configpath", help="Path to config file", type=str, default='/home/trainerai/trainerai-core/src/motion_analysis/config.yml')
    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]
    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()
    
    if args.ai == 'spin':
        pose_definition_adapter_class = SpinPoseDefinitionAdapter
    elif args.ai == 'metrabs':
        pose_definition_adapter_class = MetrabsPoseDefinitionAdapter
    else:
        rp.logerr("Could not find a suitable PoseDefinition Adapter for ai argument: <" + str(args.ai) + ">")
    
    with open(args.configpath, 'r') as infile:
        config = yaml.safe_load(infile)

    receiver = Receiver(config, pose_definition_adapter_class=pose_definition_adapter_class)

    rp.spin()

 