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
from std_msgs.msg import String
from backend.msg import Bboxes
import yaml


class getBoxesInStation:
    def __init__(self):
        result = rospy.get_param('param_server')
        self.station_dic =yaml.load(result, Loader=yaml.Loader) 
        rospy.Subscriber('exercises', String, self.setStation)
    def setStation(self,msg):
        result = rospy.get_param('param_server')
        self.station_dic =yaml.load(result, Loader=yaml.Loader)  
    def run_BoxStationChk(self,box, frame_id,stationChk):
        if stationChk == True:
            for stationID, stationXY in self.station_dic[frame_id].items(): # only stations of camera 0
                station_end_x= stationXY[0]+stationXY[2]
                station_end_y= stationXY[1]+stationXY[3]
                #print("stationCoord: ",stationXY[0], stationXY[1], station_end_x, station_end_y)
                #print("stationID: ",stationID)
                #print("-------------------------------------------------")
                #print("Detection: ",box[0], box[1], box[2], box[3])
                if (box[0]>=stationXY[0]) and (box[1]>=stationXY[1])  and (box[2]<=station_end_x) and (box[3]<=station_end_y): #X0 Y0 X1 Y1
                    return True,stationID
                
            return False,0
        else:
            return True,0
    
