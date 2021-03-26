#!/usr/bin/python3
import numpy as np
import cv2
import pafy
import matplotlib.pyplot as plt
from matplotlib import cm
from PIL import Image as ImagePil
from sensor_msgs.msg import Image
from backend.msg import Bboxes

import time
import rospy

import torch
from torch import nn
from torchvision import transforms

utils = torch.hub.load('NVIDIA/DeepLearningExamples:torchhub', 'nvidia_ssd_processing_utils')
url = "https://www.youtube.com/watch?v=wqctLW0Hb_0"




class ObjectDetectionPipeline:
    def __init__(self, threshold=0.5, device="cpu", cmap_name="tab10_r", renderer=False):
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s').eval().to(device)
        self.threshold = threshold # Confidence threshold for displaying boxes.
        self.renderer=renderer
        rospy.Subscriber('image1', Image, self.run_objectdetector)
        if self.renderer==True:
            self.publisher_img = rospy.Publisher('imageYOLO', Image , queue_size=2)
        self.publisher_boxes = rospy.Publisher('bboxes1', Bboxes , queue_size=2)
        self.body_bbox=[]
        self.spin()

    def spin(self):
            '''
            We enter in a loop and wait for exit whenever `Ctrl + C` is pressed
            '''
            rospy.spin()

    def run_objectdetector(self, img_msg):  
        shape = img_msg.height, img_msg.width, 3                            #(480, 640, 3) --> (y,x,3)
        img = np.frombuffer(img_msg.data, dtype=np.uint8)
        img_original_bgr = img.reshape(shape)
    
        tmpTime = time.time()

        img = self.obj_detect(img_original_bgr)[:,:,::-1] 

        fps = int(1/(time.time()-tmpTime))
        print("FPS : ",fps)
        if self.renderer==True:
            msg_cropImage = Image()
            msg_cropImage.header.stamp = rospy.Time.now()
            msg_cropImage.header.frame_id = 'dev0'
            msg_cropImage.encoding = "bgr8"
            msg_cropImage.data = np.array(img, dtype=np.uint8).tostring()
            msg_cropImage.height, msg_cropImage.width = img.shape[:-1]
            msg_cropImage.step = img.shape[-1]*img.shape[0]
            self.publisher_img.publish(msg_cropImage)

        array1D_body_bbox =np.array(self.body_bbox).reshape(1,-1)
        left_top  = Bboxes()
        left_top.header.stamp = img_msg.header.stamp #Will be important for data fusion: Use current time or older stamp from CameraNode
        left_top.header.frame_id = img_msg.header.frame_id #From which camera
        left_top.data=array1D_body_bbox[0]
        self.publisher_boxes.publish(left_top)

    def obj_detect(self, img):
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
        print("ges: ",resul_np )
        labels = resul_np[:,5]
        print("labels: ", labels )
        self._get_Person_boxes(labels,resul_np)
        return results.imgs[0]
    
    def _get_Person_boxes(self, labels, resul_np):
        """Plot boxes on an image"""
        self.body_bbox=[]
        for count, label in enumerate(labels):
            if label==0:
                x=resul_np[count,0]
                y=resul_np[count,1]
                w=resul_np[count,2]-x
                h=resul_np[count,3]-y
                self.body_bbox.append([x,y,w,h])
                #Change it to x,y,w,h


if __name__ == '__main__':  
    rospy.init_node('objectNodeYOLO', anonymous=True)
    # instantiate the Comparator class
    obj_detect = ObjectDetectionPipeline(device="cuda", threshold=0.2,renderer=False)                  #Later on, we can choose a specific detector. We have to write a new class for each detector
