#!/usr/bin/python3
import numpy as np
import cv2
import pafy
import matplotlib.pyplot as plt
from matplotlib import cm
from PIL import Image as ImagePil
from sensor_msgs.msg import Image
from backend.msg import Bboxes
from scheduler import getBoxesInStation
import time
import rospy
import yaml

import torch
from torch import nn
from torchvision import transforms

utils = torch.hub.load('NVIDIA/DeepLearningExamples:torchhub', 'nvidia_ssd_processing_utils')
url = "https://www.youtube.com/watch?v=wqctLW0Hb_0"


class ObjectDetectionPipeline:
    def __init__(self, threshold=0.5, device="cpu", cmap_name="tab10_r", renderer=False, stationChk=False):
        self.stationBoxesChk = getBoxesInStation()
        self.stationChk=stationChk
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s').eval().to(device)
        self.threshold = threshold # Confidence threshold for displaying boxes.
        self.renderer=renderer
        rospy.Subscriber('image', Image, self.run_objectdetector)
        if self.renderer==True:
            self.publisher_img = rospy.Publisher('imageYOLO', Image , queue_size=2)
        self.publisher_boxes = rospy.Publisher('bboxes', Bboxes , queue_size=2)
        self.body_bbox=[]
        self.info_station=[]
        self.frame_id=0
        self.spin() #Dont write any line of code after self.spin!

    def spin(self):
            '''
            We enter in a loop and wait for exit whenever `Ctrl + C` is pressed
            '''
            rospy.spin()

    def run_objectdetector(self, img_msg):  
        shape = img_msg.height, img_msg.width, 3                            #(480, 640, 3) --> (y,x,3)
        img = np.frombuffer(img_msg.data, dtype=np.uint8)
        img_original_bgr = img.reshape(shape)
        self.frame_id=int(img_msg.header.frame_id[3:])
        tmpTime = time.time()

        img = self.obj_detectYolo(img_original_bgr)[:,:,::-1] 

        fps = int(1/(time.time()-tmpTime))
        print("FPS : ",fps)
        if self.renderer==True:
            msg_renderImage = Image()
            msg_renderImage.header.stamp = rospy.Time.now()
            msg_renderImage.header.frame_id = 'dev0'
            msg_renderImage.encoding = "bgr8"
            msg_renderImage.data = np.array(img, dtype=np.uint8).tostring()
            msg_renderImage.height, msg_renderImage.width = img.shape[:-1]
            msg_renderImage.step = img.shape[-1]*img.shape[0]
            self.publisher_img.publish(msg_renderImage)
        
        array1D_body_bbox =np.array(self.body_bbox).reshape(1,-1)
        left_top  = Bboxes()
        left_top.header.stamp = img_msg.header.stamp #Will be important for data fusion: Use current time or older stamp from CameraNode
        left_top.header.frame_id = img_msg.header.frame_id #From which camera
        left_top.data=array1D_body_bbox[0]
        left_top.stationID=self.info_station  
        left_top.sensorID=self.frame_id       
        self.publisher_boxes.publish(left_top)

    def obj_detectYolo(self, img):
        """
        Now the call method This takes a raw frame from opencv finds the boxes and draws on it.
        """  
        img_tens =img   
        results = self.model(img_tens,size=640)
        if self.renderer==True:
            #results.save()
            results.render()  # updates results.imgs with boxes and labels
        
        results.imgs # array of original images (as np array) passed to model for inference
        resul_np=results.xyxy[0].cpu().detach().numpy()
        
        bbox_size =  [ ((x[2]-x[0]) * (x[3]-x[1])) for x in resul_np]
        idx_big2small = np.argsort(bbox_size)[::-1]                                
        resul_np = [ resul_np[i] for i in idx_big2small ]  
        resul_np = np.array(resul_np)
        
        labels = resul_np[:,5]
        self._get_Person_boxes(labels,resul_np)
        return results.imgs[0]
    
    def _get_Person_boxes(self, labels, resul_np):
        """Plot boxes on an image"""
        self.body_bbox=[]
        self.info_station=[]  
        tmp_conf=0
        for count, label in enumerate(labels):
            box=resul_np[count,:4]
            if label==0:
                if self.stationChk:
                    chkIsstation,stationID = self.stationBoxesChk.run_BoxStationChk(box,self.frame_id , self.stationChk)
                    if chkIsstation and (stationID not in self.info_station): #ToDo: SensorID to check the correct yaml file
                        #Change it to x,y,w,h for the PoseAI
                        x=box[0]
                        y=box[1]
                        w=box[2]-x
                        h=box[3]-y
                        self.body_bbox.append([x,y,w,h])
                        self.info_station.append(stationID)
                        print("self.info_station: ",self.info_station)
                else:
                    if resul_np[count,4]>tmp_conf:
                        self.body_bbox= [None] * 1
                        tmp_conf=resul_np[count,4]
                        tmp_x=box[0]
                        tmp_y=box[1]
                        tmp_w=box[2]-tmp_x
                        tmp_h=box[3]-tmp_y
                        self.body_bbox[0]= ([tmp_x,tmp_y,tmp_w,tmp_h])

              

if __name__ == '__main__':  
    rospy.init_node('objectNodeYOLO', anonymous=True)
    rospy.set_param('param_server', yaml.dump({0 : {}}))
    # instantiate the Comparator class
    obj_detect = ObjectDetectionPipeline(device="cuda", threshold=0.85, renderer=True, stationChk =True)                  #Later on, we can choose a specific detector. We have to write a new class for each detector
