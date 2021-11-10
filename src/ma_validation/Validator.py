#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the DataSetRecorder Node.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

import rospy as rp
import argparse
import sys
import sys
import json

from backend.msg import Persons

from ma_validation.msg import MAValidationSetInfo
from motion_analysis.src.DataConfig import ROS_TOPIC_USER_EXERCISE_STATES
from ma_validation.DataSetRecorder import MA_VALIDATION_VIDEO_TIMING_TOPIC, MA_VALIDATION_SET_TOPIC
from std_msgs.msg import Int32, String


class Validator():
    def __init__(self):
        self.station_manager = station_manager
        self._recording = True
        # Define a subscriber to retrive tracked bodies
        self.video_timing_subscriber = rp.Subscriber(MA_VALIDATION_VIDEO_TIMING_TOPIC, Int32, self.video_timing_callback)
        self.ma_validation_set_subscriber = rp.Subscriber(MA_VALIDATION_SET_TOPIC, MAValidationSetInfo, self.ma_validation_set_callback)
        self.exercise_states_subscriber = rp.Subscriber(ROS_TOPIC_USER_EXERCISE_STATES, String, self.exercise_state_callback)

        self.active_set = None
        self.last_set = None

        def exercise_state_callback(self, msg):
            data = json.loads(msg)
            t0 = data["seconds_since_last_exercise_start

        def ma_validation_set_callback(self, msg):
            if message.start:
                self.active_set_= msg
                rp.logerr("Started new set: " + str(msg))
            else:
                rp.logerr("Endet new set: " + str(msg))
                self.last_set = msg
                time.sleep
                set_message = MAValidationSetInfo()
                set_message.exercise_id = current_set["exercise_id"]
                set_message.t_from_s = current_set["t_from_s"]
                set_message.t_to_s = current_set["t_to_s"]
                set_message.reps = current_set["reps"]
                set_message.start = True
                self.ma_validation_set_publisher.publish(set_message)

                rp.logerr("Started set: " + str(current_set))

                station_manager.start_exercise(self.sm_client_id, DEBUG_STATION_ID, current_set["exercise_id"])

                self.active_set = current_set

            if not self.set_list:
                rp.signal_shutdown("All sets finished. Stopping recording...")


if __name__ == '__main__':
    rp.init_node('DataSetRecorder', anonymous=False)
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--debug", help="Only one camera with QT selection", action="store_true")
    parser.add_argument("-v", "--verbose", help="Verbose mode", action="store_true")
    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]
    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()


    def signal_handler():
        recorder._recording = False

    rp.on_shutdown(signal_handler)
    
    rp.spin()

    

