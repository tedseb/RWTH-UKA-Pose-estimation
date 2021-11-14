#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the DataSetRecorder Node.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

import rospy as rp
import numpy as np
import argparse
import sys
import sys
import json
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

from backend.msg import Persons

from ma_validation.msg import MAValidationSetInfo
from DataSetRecorder import MA_VALIDATION_VIDEO_TIMING_TOPIC, MA_VALIDATION_SET_TOPIC
from std_msgs.msg import Int32, String

# Make use of rosbag play
# --rate-control-topic=RATE_CONTROL_TOPIC

# watch the given topic, and if the last publish was more than <rate-control-max-delay> ago, wait until the topic publishes again to continue playback

# --rate-control-max-delay=RATE_CONTROL_MAX_DELAY

# maximum time difference from <rate-control-topic> before pausing


class Validator():
    def __init__(self):
        # Define a subscriber to retrive tracked bodies
        # self.video_timing_subscriber = rp.Subscriber(MA_VALIDATION_VIDEO_TIMING_TOPIC, Int32, self.video_timing_callback)
        self.ma_validation_set_subscriber = rp.Subscriber(MA_VALIDATION_SET_TOPIC, MAValidationSetInfo, self.ma_validation_set_callback)
        self.exercise_states_subscriber = rp.Subscriber('user_state', String, self.exercise_state_callback)

        self.active_set = MAValidationSetInfo()
        self.active_set_reps = None
        self.last_set = MAValidationSetInfo()
        self.last_set_reps = None

        self.total_reps = 0
        self.positive_errors = 0
        self.negative_errors = 0
        self.done_exercises = {}

    def exercise_state_callback(self, msg):
        data = json.loads(msg.data)['data']
        station_usage_hash = str(data['station_usage_hash'])
        if station_usage_hash == str(self.last_set.station_usage_hash):
            self.last_set_reps = data["repetitions"]
        elif station_usage_hash == str(self.active_set.station_usage_hash):
            self.active_set_reps = data["repetitions"]
        else:
            rp.logerr("Could not match current station_usage_hash with the one that we got from the motion analysis.")

    def ma_validation_set_callback(self, msg):
        if msg.start:
            self.last_set = self.active_set
            self.last_set_reps = self.active_set_reps
            self.active_set = msg
            self.active_set_reps = 0
            rp.logerr("Started set")
        else:
            finished_set = self.active_set
            rp.logerr("Finished set")
            if self.active_set_reps == 0:
                rp.logerr("Set with 0 reps not counted - this exercise is probably not supported by system and therefore not counted.")
                return

            positive_error = np.clip(self.active_set_reps - msg.reps, 0, None)
            negative_error = np.clip(msg.reps - self.active_set_reps, 0, None)
            if not finished_set.exercise_id in self.done_exercises.keys():
                self.done_exercises[finished_set.exercise_id] = finished_set
            self.done_exercises[finished_set.exercise_id].reps += finished_set.reps # Use this at the end of the test
            rp.logerr("Positive Error: " + str(positive_error)) 
            rp.logerr("Negative Error: " + str(negative_error)) 
            self.positive_errors += positive_error
            self.negative_errors += negative_error
            rp.logerr("Total positive errors: " + str(self.positive_errors))
            rp.logerr("Total negative errors: " + str(self.negative_errors))
            rp.logerr("Total error: " + str(self.positive_errors + self.negative_errors))

    def create_report(self, df):
        fig, ax = plt.subplots()

        # hide axes
        fig.patch.set_visible(False)
        ax.axis('off')
        ax.axis('tight')

        df = pd.DataFrame(np.random.randn(10, 4), columns=list('ABCD'))

        ax.table(cellText=df.values, colLabels=df.columns, loc='center')

        fig.tight_layout()

        pp = PdfPages('multipage.pdf')
        pp.savefig()
        pp.close()


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

    

