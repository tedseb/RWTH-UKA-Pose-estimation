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
import sched, time
import yaml
import subprocess
 
from msg import MAValidation

from station_manager import DEBUG_STATION_ID


class AnalysisValidator():
    def __init__(self, 
    rosbag_file_path: str):
        self.validation_publisher = rp.Subscriber("ma_validation_sets", MAValidation, self.callback)

        if pathlib.Path(input_video_file_path).name != timecode_data.file_name:
            rp.logerr("Input video file name does not match file name specified in timecode file")

        def get_t_from(list_item):
            return list_item.t_from

        self.set_list = timecode_data.sets.sort(key=get_t_from)
        self.done_sets = []
        self.active_set = None


    def callback(self, message: MAValidation) -> None:
        pass

if __name__ == '__main__':
    rp.init_node('DataSetRecorder', anonymous=False)

    signal.signal(signal.SIGINT, signal_handler)
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--debug", help="Only one camera with QT selection", action="store_true")
    parser.add_argument("-v", "--verbose", help="Verbose mode", action="store_true")
    parser.add_argument("-r", "--rosbag_file_path", help="The rosbag file needed for evaluation", default=None)
    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]
    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()

    validator = AnalysisValidator(rosbag_file_path)

    rp.spin()

    # TODO: Start Motion Analysis and ROSBag file