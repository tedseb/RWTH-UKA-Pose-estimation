#!/usr/bin/python3
import os
import sys
import os.path as osp
import numpy as np
import cv2
import json
import pickle
import time
import rospy
from sensor_msgs.msg import Image
from backend.msg import Bboxes
import yaml


class getBoxesInStation:
    def __init__(self):
        self.station_dic=[]
        self.run_BoxStationChk([])
        if os.path.exists('/home/trainerai/trainerai-core/src/infrastructure/stations/dev0.yaml'):
            with open(r'/home/trainerai/trainerai-core/src/infrastructure/stations/dev0.yaml') as file:
                self.station_dic = yaml.load(file,Loader=yaml.Loader)   #Cam0

    def run_BoxStationChk(self,box):
        current_exercise = yaml.safe_load(rospy.get_param('exercise'))
        print(self.station_dic)
        for stationID in [3]: # only station 3
            station_end_x= self.station_dic[station][2]
            station_end_y= self.station_dic[station][3]
            if (box[0]>=self.station_dic[station][0]) and (detection[1]>=self.station_dic[station][1])  and (box[2]<=station_end_x) and (box[3]<=station_end_y): #X0 Y0 X1 Y1
                return True
        return False

obj_detect=getBoxesInStation()