#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the DataSetRecorder Node.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

import rospy as rp
from typing import Any
import signal
import argparse
import sys
import pathlib
 
from station_manager import StationManager, signal_handler

from backend.msg import Persons


class DataSetRecorder():
    def __init__(self, 
    station_manager: StationManager):
        self.station_manager = station_manager
        # Define a subscriber to retrive tracked bodies
        rp.Subscriber(ROS_JOINTS_TOPIC, Persons, self.callback)
        self.spot_queue_interface = spot_queue_interface_class()

        self.spot_queue_load_balancer = spot_queue_load_balancer_class()
        self.feature_extractor = feature_extractor_class()

    def callback(self, message: Any) -> None:
        pass
        

if __name__ == '__main__':
    rp.init_node('DataSetRecorder', anonymous=False)

    signal.signal(signal.SIGINT, signal_handler)
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--debug", help="Only one camera with QT selection", action="store_true")
    parser.add_argument("-v", "--verbose", help="Verbose mode", action="store_true")
    parser.add_argument("-i", "--input", help="Input directory", action="store_true")
    parser.add_argument("-o", "--output", help="Output file", action="store_true")
    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]
    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()

    camera_path = str(pathlib.Path(__file__).absolute().parent.parent) + "/infrastructure/CameraNode.py"
    transform_node_path = str(pathlib.Path(__file__).absolute().parent.parent) + "/station_manager/launch/static_transform.launch"
    station_selection_path = str(pathlib.Path(__file__).absolute().parent.parent) + "/station_manager/src/station_selection.py"

    station_manager = StationManager(camera_path, transform_node_path, station_selection_path, debug_mode=args.debug, verbose=args.verbose)

    DataSetRecorder = DataSetRecorder(station_manager)

    rp.spin()