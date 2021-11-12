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
from DataSetRecorder import MA_VALIDATION_VIDEO_TIMING_TOPIC, MA_VALIDATION_SET_TOPIC
from std_msgs.msg import Int32, String


class Validator():
    def __init__(self):
        # Define a subscriber to retrive tracked bodies
        # self.video_timing_subscriber = rp.Subscriber(MA_VALIDATION_VIDEO_TIMING_TOPIC, Int32, self.video_timing_callback)
        self.ma_validation_set_subscriber = rp.Subscriber(MA_VALIDATION_SET_TOPIC, MAValidationSetInfo, self.ma_validation_set_callback)
        self.exercise_states_subscriber = rp.Subscriber('user_state', String, self.exercise_state_callback)

        self.active_set = None
        self.active_set_reps = None
        self.last_set = None
        self.last_set_reps = None
        self.station_usage_hashes_changed_flag = False

    def push_set(self, msg):
        self.last_set = self.active_set
        self.last_set_reps = self.active_set_reps

    def exercise_state_callback(self, msg):
        data = json.loads(msg)
        station_usage_hash = data.get('station_usage_hash')
        if station_usage_hash == self.last_set['station_usage_hash']:
            self.last_set["reps"] = data["repetitions"]
        elif station_usage_hash == self.active_set['station_usage_hash']:
            self.active_set["reps"] = data["repetitions"]
        else:
            # A new exercise has started
            self.last_set_station_usage_hash = self.active_set_station_usage_hash
            self.active_set_station_usage_hash = station_usage_hash
            self.active_set["reps"] = data["repetitions"]
            rp.logerr("Could not match current station_usage_hash with the one that we got from the motion analysis.")

    def ma_validation_set_callback(self, msg):
        station_usage_hash = data.get('station_usage_hash')
        if msg.start:
            push_set(msg)
            rp.logerr("Started  ` set: " + str(msg))
        else:
            if station_usage_hash == self.active_set['station_usage_hash']:
                finished_set = self.active_set
            elif station_usage_hash == self.last_set['station_usage_hash']:
                finished_set = self.last_set
            rp.logerr("Finished set with " + str(msg["reps"] + " out of " + str(finished_set["reps"]) + " repetitions."))
              

            rp.logerr("Started set: " + str(current_set))



if __name__ == '__main__':
    rp.init_node('Validator', anonymous=False)
    
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

    validator = Validator()
    
    rp.spin()

    

