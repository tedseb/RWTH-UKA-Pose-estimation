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
 
from station_manager import StationManager, signal_handler, DEBUG_STATION_ID

from ma_validation.msg import MAValidationSetInfo

class DataSetRecorder():
    def __init__(self,
    station_manager: StationManager,
    input_video_file_path: str = "/home/trainerai/trainerai-core/data/video.mp4",
    timecode_file_path: str = "/home/trainerai/trainerai-core/data/timecodes.yml",
    output_file_path: str = "/home/trainerai/ma_validation_recorder_output.bag"):
        self.station_manager = station_manager
        # Define a subscriber to retrive tracked bodies
        self.validation_set_publisher = rp.Publisher("ma_validation_sets", MAValidation, queue_size=100)
        self.video_timing_subscriber = rp.Subscriber("ma_validation_video_timing", float, self.callback)

        with open(timecode_file_path) as stream:
            try:
                timecode_data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return
        
        if pathlib.Path(input_video_file_path).name != timecode_data.file_name:
            rp.logerr("Input video file name does not match file name specified in timecode file")

        def get_t_from(list_item):
            return list_item.t_from

        self.set_list = timecode_data.sets.sort(key=get_t_from)
        self.done_sets = []
        self.active_set = None


        def repetition_callback(response_code=508, satus_code=2, payload=dict({})):
            # TODO: Why do we need this?
            print("repetitions =", payload["repetitions"])
            print("exercise =", payload["exercise"])
            print("set id =", payload["set_id"])

        self.sm_client_id = "ma_validation_dataset_recorder"
        self.station_manager.set_client_callback(my_id, repetition_callback)
        self.station_manager.login_station(self.sm_client_id, 2)


    def callback(self, t: float) -> None:
        def are_we_past_t_from(_set):
            if t > _set.t_from:
                return True
            else:
                return False

        if are_we_past_t_from(self.active_set):
            set_message = MAValidation()
            set_message.exercise_id = current_set.exercise_id
            set_message.t_from = current_set.t_from
            set_message.t_to = current_set.t_to
            set_message.repetitions = current_set.repetitions
            set_message.start = False
            self.active_set = None

            station_manager.stop_exercise(self.sm_client_id)

        while are_we_past_t_from(self.set_list[0]):
            current_set = self.set_list.pop(0)
            set_message = MAValidation()
            set_message.exercise_id = current_set.exercise_id
            set_message.t_from = current_set.t_from
            set_message.t_to = current_set.t_to
            set_message.repetitions = current_set.repetitions
            set_message.start = True
            self.validation_set_publisher.publish(set_message)

            station_manager.start_exercise(self.sm_client_id, DEBUG_STATION_ID, exercise_id)

            self.active_set = current_set

            station_manager.logout_station(self.sm_client_id)


if __name__ == '__main__':
    rp.init_node('DataSetRecorder', anonymous=False)

    signal.signal(signal.SIGINT, signal_handler)
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--debug", help="Only one camera with QT selection", action="store_true")
    parser.add_argument("-v", "--verbose", help="Verbose mode", action="store_true")
    parser.add_argument("-i", "--input", help="Input directory", default=None)
    parser.add_argument("-o", "--output", help="Output file", default=None)
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

    DataSetRecorder = DataSetRecorder(station_manager, input_folder=args.input, output_file=parser.out)

    rp.spin()

    # TODO: Publish video timing somehow, so that the dataset recorder can time the exercises
    # self.video_timing_publisher = rp.Publisher("ma_validation_video_timing", float, queue_size=100)
