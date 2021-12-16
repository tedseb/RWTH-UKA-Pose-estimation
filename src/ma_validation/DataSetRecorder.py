#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the DataSetRecorder Node.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

import rospy as rp
import argparse
import sys
import pathlib
import yaml
import rosbag
from collections import deque
import sys
import os
import logy
import time

sys.path.append('/home/trainerai/trainerai-core/')
from src.station_manager.src import DataManager
from src.station_manager.src.station_manager import StationManager, DEBUG_STATION_ID
 
from ma_validation.msg import MAValidationSetInfo
from std_msgs.msg import Int32

MA_VALIDATION_VIDEO_TIMING_TOPIC = "ma_validation_video_timing"
MA_VALIDATION_SET_TOPIC = "ma_validation_set"

class DataSetRecorder():
    def __init__(self,
    station_manager: StationManager,
    input_video: str = "/home/trainerai/trainerai-core/data/videos/video.mp4",
    input_timecodes: str = "/home/trainerai/trainerai-core/data/videos/timecodes.yml",
    output_file: str = "/home/trainerai/trainerai-core/data/videos/ma_validation_recorder_output.bag"):
        logy.Logy().basic_config(debug_level=logy.DEBUG, module_name="SM")
        self.station_manager = station_manager
        self._recording = True
        # Define a subscriber to retrive tracked bodies
        self.video_timing_subscriber = rp.Subscriber(MA_VALIDATION_VIDEO_TIMING_TOPIC, Int32, self.video_timing_callback)
        self.ma_validation_set_publisher = rp.Publisher(MA_VALIDATION_SET_TOPIC, MAValidationSetInfo, queue_size=100)
        self.msg_queue = deque()

        self.output_file = output_file

        with open(input_timecodes) as stream:
            try:
                timecode_data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return
        
        if pathlib.Path(input_video).name != timecode_data["file_name"]:
            logy.warning("Input video file name does not match file name specified in timecode file")

        def get_t_from(list_item):
            return list_item["t_from_s"]

        self.set_list = timecode_data["sets"]
        self.set_list.sort(key=get_t_from)
        self.done_sets = []
        self.active_set = None

        def repetition_callback(response_code=508, satus_code=2, payload=dict({})):
            # TODO: Why do we need this?
            print("repetitions =", payload["repetitions"])
            print("exercise =", payload["exercise"])
            print("set id =", payload["set_id"])

        self.sm_client_id = "ma_validation_dataset_recorder"
        self.station_manager.set_client_callback(self.sm_client_id, repetition_callback)
        self.station_manager.login_station(self.sm_client_id, DEBUG_STATION_ID)

    def video_timing_callback(self, msg: Int32) -> None:
        """ Publishes validation set messages.
        
        For every timestep in the video, published by the camera node, check if we have a set to start.
        """
        t = msg.data

        if self.active_set and t > self.active_set["t_to_s"]:
            _, station_usage_hash = station_manager.stop_exercise(self.sm_client_id)

            set_message = MAValidationSetInfo()
            set_message.exercise_id = self.active_set["exercise_id"]
            set_message.t_from_s = self.active_set["t_from_s"]
            set_message.t_to_s = self.active_set["t_to_s"]
            set_message.reps = self.active_set["reps"]
            set_message.start = False
            set_message.station_usage_hash = station_usage_hash
            
            self.ma_validation_set_publisher.publish(set_message)

            logy.debug("Ended set: " + str(self.active_set))
            self.active_set = None

        if self.set_list and t > self.set_list[0]["t_from_s"]:
            current_set = self.set_list.pop(0)
            logy.debug("Started set: " + str(current_set))
            _, station_usage_hash = station_manager.start_exercise(self.sm_client_id, DEBUG_STATION_ID, current_set["exercise_id"])

            set_message = MAValidationSetInfo()
            set_message.exercise_id = current_set["exercise_id"]
            set_message.t_from_s = current_set["t_from_s"]
            set_message.t_to_s = current_set["t_to_s"]
            set_message.reps = current_set["reps"]
            set_message.start = True
            set_message.station_usage_hash = station_usage_hash
            self.ma_validation_set_publisher.publish(set_message)

            self.active_set = current_set

        if (not self.set_list and not self.active_set)or not self._recording:
            try:
                rp.logerr("Waiting for 10 seconds for last messagse to roll in before ending recording...")
                time.sleep(10)
                os.system("rosnode kill Bagfilerecorder &")
                os.system("rosnode kill DataSetRecorder &")
                del self.station_manager # This spawns a routine that ends the camera node process
                rp.logerr("Ignore warning about Bagfilerecorder if present...")
            except AttributeError:
                pass
            
    

if __name__ == '__main__':
    rp.init_node('DataSetRecorder', anonymous=False)
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--debug", help="Only one camera with QT selection", action="store_true")
    parser.add_argument("-v", "--verbose", help="Verbose mode", action="store_true")
    parser.add_argument("-i", "--input_video", help="Input video", default=None)
    parser.add_argument("-t", "--input_timecodes", help="Input timecodes", default=None)
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
    station_selection_path = str(pathlib.Path(__file__).absolute().parent.parent) + "/station_manager/station_selection.py"

    # Wait some time to initialize pose estimation
    time.sleep(6)
    data_manager = DataManager()    
    station_manager = StationManager(camera_path, transform_node_path, station_selection_path, data_manager=data_manager, verbose=True, with_gui=False)

    recorder = DataSetRecorder(station_manager, input_video=args.input_video, input_timecodes=args.input_timecodes, output_file=args.output)
    
    rp.spin()

    

