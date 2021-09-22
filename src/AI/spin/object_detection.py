#!/usr/bin/python3
import os
import sys
import os.path as osp
import torch
from torchvision.transforms import Normalize
import numpy as np
import cv2
import json
import pickle
import time
import rospy
from sensor_msgs.msg import Image
from backend.msg import Bboxes
from bodymocap.utils.imutils import crop, crop_bboxInfo, process_image_bbox, process_image_keypoints, bbox_from_keypoints
from bodymocap import constants
import yaml



import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

from demo.demo_options import DemoOptions
from bodymocap.body_bbox_detector import BodyPoseEstimator
import mocap_utils.general_utils as gnu
from mocap_utils.timer import Timer

import renderer.image_utils as imu
from renderer.viewer2D import ImShow


def run_objectdetector(img_msg):
    shape = img_msg.height, img_msg.width, 3                            #(480, 640, 3) --> (y,x,3)
    img = np.frombuffer(img_msg.data, dtype=np.uint8)
    img_original_bgr = img.reshape(shape)
  
    tmpTime = time.time()
    
    body_bbox=[]
    info_station=[]

    body_pose_list, body_bbox_list = body_bbox_detector.detect_body_pose(img_original_bgr)
    bbox_list = [None, ] * len(body_bbox_list)
    print("Box: ", body_bbox_list)
    #Sort the bbox using bbox size 
    # (to make the order as consistent as possible without tracking)           
    bbox_size =  [ (x[2] * x[3]) for x in body_bbox_list]
    idx_big2small = np.argsort(bbox_size)[::-1]                                
    body_bbox_list = [ body_bbox_list[i] for i in idx_big2small ]              
    
    left_top  = Bboxes()

    if with_station==True:
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
        if single_person and len(body_bbox_list)>0:
            body_bbox = [body_bbox_list[0], ]  
            array1D_body_bbox=np.array(body_bbox).reshape(1,-1)
            left_top.data=array1D_body_bbox[0]
            left_top.stationID=info_station
            left_top.header.stamp = img_msg.header.stamp #Will be important for data fusion: Use current time or older stamp from CameraNode
            left_top.header.frame_id = img_msg.header.frame_id #From which camera
            publisher_crop.publish(left_top)
        else:       
            for detection in body_bbox_list: 
                body_bbox.append(detection) #In eine Stration kann nur eine Person sein, Darum das wahrs
            array1D_body_bbox=np.array(body_bbox).reshape(1,-1)
            left_top.data=array1D_body_bbox[0]
            left_top.stationID=info_station
            left_top.header.stamp = img_msg.header.stamp #Will be important for data fusion: Use current time or older stamp from CameraNode
            left_top.header.frame_id = img_msg.header.frame_id #From which camera
            publisher_crop.publish(left_top)
 
    #ToDo: separate opject detection module node
    #ToDo: differ between someone that is focused on the station and someone that is going through the camera and let to occlusion. Currently take the skeleton that is the biggest

    # fps = int(1/(time.time()-tmpTime))
    # print("FPS : ",fps)


def myhook():
    rospy.loginfo("Shutdown")
    rospy.signal_shutdown("Darum")
 
if __name__ == '__main__':
    global body_bbox_detector
    global single_person
    global with_station
    global station_dic
    
    # Set bbox detector
    body_bbox_detector = BodyPoseEstimator()
    single_person=True 
    with_station=False

    if os.path.exists('/home/trainerai/trainerai-core/src/infrastructure/stations/station_coordinate.yaml'):
        with open(r'/home/trainerai/trainerai-core/src/infrastructure/stations/station_coordinate.yaml') as file:
            station_dic = yaml.load(file,Loader=yaml.Loader)

    rospy.init_node('objectNode', anonymous=True)
    rospy.Subscriber('image', Image, run_objectdetector)
    publisher_crop = rospy.Publisher('bboxes', Bboxes , queue_size=2)
    rospy.on_shutdown(myhook)
    rospy.spin()