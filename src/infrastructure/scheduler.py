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


class getBoxesInStation():
    def __init__(self,with_station,single_person):
        self.with_station=with_station
        self.single_person=single_person
        self.station_dic=[]
        if os.path.exists('/home/trainerai/trainerai-core/src/infrastructure/stations/dev0.yaml'):
            with open(r'/home/trainerai/trainerai-core/src/infrastructure/stations/dev0.yaml') as file:
                self.station_dic = yaml.load(file,Loader=yaml.Loader)


    def run_objectdetector(self,box):
        body_bbox=[]
        info_station=[]
        
        left_top  = Bboxes()

        if self.with_station==True:
            current_exercise = yaml.safe_load(rospy.get_param('exercise'))
            for detection in body_bbox_list: 
                for station in current_exercise: #datection list:  [396  51 161 352] = [x  y w h]
                    station=float(station)
                    detection_end_x=detection[2]+detection[0]
                    detection_end_y=detection[3]+detection[1]
                    station_end_x= station_dic[station][2]+station_dic[station][0]
                    station_end_y= station_dic[station][3]+station_dic[station][1]
                    if (detection[0]>=station_dic[station][0]) and (detection[1]>=station_dic[station][1])  and (detection_end_x<=station_end_x) and (detection_end_y<=station_end_y): #X0 Y0 X1 Y1
                        body_bbox.append(detection) #In eine Stration kann nur eine Person sein, Darum das wahrs
                        info_station.append(station)
        
            array1D_body_bbox =np.array(body_bbox).reshape(1,-1)

            left_top.header.stamp = img_msg.header.stamp #Will be important for data fusion: Use current time or older stamp from CameraNode
            left_top.header.frame_id = img_msg.header.frame_id #From which camera
            left_top.data=array1D_body_bbox[0]
            left_top.stationID=info_station
            publisher_crop.publish(left_top)

        else:
            if self.single_person:
                body_bbox = box
            else:       
                for detection in body_bbox_list: 
                    body_bbox.append(detection) #In eine Stration kann nur eine Person sein, Darum das wahrs
                array1D_body_bbox=np.array(body_bbox).reshape(1,-1)
                left_top.data=array1D_body_bbox[0]
                left_top.stationID=info_station
                left_top.header.stamp = img_msg.header.stamp #Will be important for data fusion: Use current time or older stamp from CameraNode
                left_top.header.frame_id = img_msg.header.frame_id #From which camera
                publisher_crop.publish(left_top)
    return body_bbox