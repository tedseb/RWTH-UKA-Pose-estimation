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
from backend.msg import Person, Persons, Bodypart, Pixel,Bboxes
from std_msgs.msg import Float32MultiArray

import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

from demo.demo_options import DemoOptions
from bodymocap.body_mocap_api import BodyMocap
import mocap_utils.general_utils as gnu
from mocap_utils.timer import Timer

import renderer.image_utils as imu
from renderer.viewer2D import ImShow


class run_spin():
    def __init__(self):
        device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        assert torch.cuda.is_available(), "Current version only supports GPU"

        # Set mocap regressor
        use_smplx = False

        checkpoint_body_smplx='/home/trainerai/trainerai-core/src/AI/spin/extra_data/body_module/pretrained_weights/smplx-03-28-46060-w_spin_mlc3d_46582-2089_2020_03_28-21_56_16.pt'
        checkpoint_body_smpl= '/home/trainerai/trainerai-core/src/AI/spin/extra_data/body_module/pretrained_weights/2020_05_31-00_50_43-best-51.749683916568756.pt'
        smpl_dir='/home/trainerai/trainerai-core/src/AI/spin/extra_data/smpl/'

        checkpoint_path = checkpoint_body_smplx if use_smplx else checkpoint_body_smpl
        print("use_smplx", use_smplx)
        self.body_mocap = BodyMocap(checkpoint_path, smpl_dir, device, use_smplx)

        # define a publisher to publish the 3D skeleton of multiple people
        self.publisher = rospy.Publisher('personsJS', Persons, queue_size=2)
        self.publisher_crop = rospy.Publisher('cropImage', Image, queue_size=2)   


        # define a subscriber to retrive tracked bodies
        rospy.Subscriber('bboxes', Bboxes, self.callback_regress)
        #rospy.Subscriber('bboxes1', Bboxes, self.callback_regress)

        rospy.Subscriber('image', Image, self.callback_setImage)       
        #srospy.Subscriber('image1', Image, self.callback_setImage)
        self.spin()


    def spin(self):
        '''
        We enter in a loop and wait for exit whenever `Ctrl + C` is pressed
        '''
        rospy.spin()

    def callback_setImage(self, msg):
        self.msg_image = msg
        shape = self.msg_image.height, self.msg_image.width, 3     #(480, 640, 3) --> (y,x,3) = (h,w,3)
        img = np.frombuffer(self.msg_image.data, dtype=np.uint8)
        self.img_original_bgr = img.reshape(shape)

    def callback_regress(self, body_bbox_list_station):
        '''
        This function will be called everytime whenever a message is received by the subscriber
        '''
        body_bbox_list_station_reshaped=np.array(body_bbox_list_station.data).reshape(-1,4)
        tmpTime = time.time()
 
        #ToDo: differ between someone that is focused on the station and someone that is going through the camera and let to occlusion. Currently take the skeleton that is the biggest

        # Body Pose Regression
        pred_output_list = self.body_mocap.regress(self.img_original_bgr, body_bbox_list_station_reshaped)
        #x= pred_output_list
        fps = int(1/(time.time()-tmpTime))
        #print("result: ", np.shape(pred_output_list.extend(x)))
        print("FPS : ",fps)
        self.publish_results(pred_output_list, self.msg_image,  body_bbox_list_station.stationID,body_bbox_list_station.sensorID )


    def publish_results(self,results, img_msg, stationID,sensorID):  
        if len(results) == 0:
            return
        inc=0
        msg = Persons()
        msg.header = img_msg.header
        msg.persons = list()

        cropImage=[]
        for detection in results:
            joints=detection["pred_joints_img"]
            cropImage.append(detection['img_cropped'])
        
            lenPoints=len(joints)       #ToDo use fix number of joints to save calculation time
            person_msg = Person()
            if len(stationID) >0:
                person_msg.stationID = int(stationID[inc])
                person_msg.sensorID = sensorID[inc]
            person_msg.bodyParts = [None]*lenPoints
            for idx in range(lenPoints):
                person_msg.bodyParts[idx] = Bodypart()
            for idx in range(lenPoints):
                person_msg.bodyParts[idx].score = 0.8
                person_msg.bodyParts[idx].pixel.x = joints[idx,0]
                person_msg.bodyParts[idx].pixel.y = joints[idx,1]
                person_msg.bodyParts[idx].point.x =joints[idx,0]/200-3
                person_msg.bodyParts[idx].point.y =joints[idx,2]/200
                person_msg.bodyParts[idx].point.z = -joints[idx,1]/200+3.2          #ToDo: Normalisierung ins Backend
            inc=inc+1
            msg.persons.append(person_msg)
        self.publisher.publish(msg)
        
        imgAdd=cv2.vconcat(cropImage)  
        msg_cropImage = Image()
        msg_cropImage.header.stamp = rospy.Time.now()
        msg_cropImage.header.frame_id = img_msg.header.frame_id
        msg_cropImage.encoding = "bgr8"
        msg_cropImage.data = np.array(imgAdd, dtype=np.uint8).tostring()
        msg_cropImage.height, msg_cropImage.width = imgAdd.shape[:-1]
        msg_cropImage.step = imgAdd.shape[-1]*imgAdd.shape[0]

        self.publisher_crop.publish(msg_cropImage)


if __name__ == '__main__':



    rospy.init_node('spin', anonymous=True)
    # instantiate the Comparator class
    run_spin_obj = run_spin()
