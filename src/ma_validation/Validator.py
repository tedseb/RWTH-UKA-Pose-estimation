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
import threading

import datetime

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
        self.ma_validation_done_subscriber = rp.Subscriber('ma_validation_done', Int32, self.create_report)

        self.active_set = MAValidationSetInfo()
        self.active_set_reps = 0
        self.last_set = MAValidationSetInfo()
        self.last_set_reps = 0

        self.semaphore = threading.Semaphore()

        self.total_reps = 0
        self.positive_errors = 0
        self.negative_errors = 0
        self.done_exercises = {}

    def exercise_state_callback(self, msg):
        data = json.loads(msg.data)['data']
        station_usage_hash = str(data['station_usage_hash'])
        if station_usage_hash == str(self.last_set.station_usage_hash):
            rp.logerr(msg)
            rp.logerr(self.last_set)
            self.last_set_reps = data["repetitions"]
        elif station_usage_hash == str(self.active_set.station_usage_hash):
            rp.logerr(msg)
            rp.logerr(self.active_set)
            self.active_set_reps = data["repetitions"]
        else:
            rp.logerr("Could not match current station_usage_hash with the one that we got from the motion analysis.")

    def ma_validation_set_callback(self, msg):
        self.semaphore.acquire()
        rp.logerr(msg)
        rp.logerr(self.last_set)
        rp.logerr(self.active_set)
        if msg.start:
            self.last_set = self.active_set
            self.last_set_reps = self.active_set_reps
            self.active_set = msg
            self.active_set_reps = 0
            rp.logerr("Started set")
        else:
            if msg.t_from_s == self.active_set.t_from_s:
                finished_set = self.active_set
                reps = self.active_set_reps
            elif msg.t_from_s == self.last_set.t_from_s:
                finished_set = self.last_set
                reps = self.last_set_reps
            else:
                rp.logerr("Could not match current station_usage_hash with the one that we got from the motion analysis.")
                return
            rp.logerr("Finished set")

            positive_error = np.clip(reps - msg.reps, 0, None)
            negative_error = np.clip(msg.reps - reps, 0, None)
            rp.logerr("Positive Error: " + str(positive_error)) 
            rp.logerr("Negative Error: " + str(negative_error)) 
            self.positive_errors += positive_error
            self.negative_errors += negative_error
            rp.logerr("Total positive errors: " + str(self.positive_errors))
            rp.logerr("Total negative errors: " + str(self.negative_errors))
            rp.logerr("Total error: " + str(self.positive_errors + self.negative_errors))

            # We update this dictionary from which we build the report in the end
            if not finished_set.exercise_id in self.done_exercises.keys():
                self.done_exercises[finished_set.exercise_id] = np.array([0, 0, 0, 0, 0])

            self.done_exercises[finished_set.exercise_id] = self.done_exercises[finished_set.exercise_id] + np.array([msg.reps, 0, positive_error + negative_error, positive_error, negative_error])
        
        self.semaphore.release()
            
    def create_report(self, msg):
        rows = list(self.done_exercises.keys())
        columns = ["Total Repetitions", "Percent Counted", "Total Error", "Positive Error", " Negative Error"]
        df = pd.DataFrame.from_dict(self.done_exercises, orient='index', columns=columns) # (np.random.randn(10, 4), columns=list('ABCD'))
        with PdfPages('/home/trainerai/trainerai-core/data/ma_validation_report.pdf') as pdf:
            fig, ax = plt.subplots()
            fig.patch.set_visible(False)
            ax.axis('off')
            ax.axis('tight')
            ax.table(cellText=df.values, colLabels=df.columns, loc='center', rowLabels=rows)

            plt.title('Motion Analysis Validation Report \nby Exercise ID')
            pdf.savefig()
            plt.close()

            d = pdf.infodict()
            d['Title'] = 'Multipage PDF Example'
            d['Author'] = 'Artur Niederfahrenhorst'
            d['Subject'] = 'Motion Analysis Validation Report'
            d['CreationDate'] = datetime.datetime.today()
            d['ModDate'] = datetime.datetime.today()



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
