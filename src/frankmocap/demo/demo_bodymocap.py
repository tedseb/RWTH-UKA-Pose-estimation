# Copyright (c) Facebook, Inc. and its affiliates.

import os
import sys
import os.path as osp
import torch
from torchvision.transforms import Normalize
import numpy as np
import cv2
import argparse
import json
import pickle
import time
import rospy
from sensor_msgs.msg import Image
from pose_estimation.msg import Person, Persons, Bodypart, Pixel

from demo.demo_options import DemoOptions
from bodymocap.body_mocap_api import BodyMocap
from bodymocap.body_bbox_detector import BodyPoseEstimator
import mocap_utils.general_utils as gnu
from mocap_utils.timer import Timer

import renderer.image_utils as imu
from renderer.viewer2D import ImShow


def run_body_mocap(img_msg):
    shape = img_msg.height, img_msg.width, 3
    img = np.frombuffer(img_msg.data, dtype=np.uint8)
    img_original_bgr = img.reshape(shape)
  


    tmpTime = time.time()
    
    
    body_pose_list, body_bbox_list = body_bbox_detector.detect_body_pose(img_original_bgr)
    hand_bbox_list = [None, ] * len(body_bbox_list)

    #Sort the bbox using bbox size 
    # (to make the order as consistent as possible without tracking)
    bbox_size =  [ (x[2] * x[3]) for x in body_bbox_list]
    idx_big2small = np.argsort(bbox_size)[::-1]
    body_bbox_list = [ body_bbox_list[i] for i in idx_big2small ]
    if args.single_person and len(body_bbox_list)>0:
        body_bbox_list = [body_bbox_list[0], ]       

    # Body Pose Regression
    pred_output_list = body_mocap.regress(img_original_bgr, body_bbox_list)
    assert len(body_bbox_list) == len(pred_output_list)
    fps = int(1/(time.time()-tmpTime))

    print("FPS: ",fps)

    publish_results(pred_output_list,img_msg)

def publish_results(results, img_msg):  
    if len(results) == 0:
        return
    msg = Persons()
    msg.header = img_msg.header
    msg.persons = list()
    for detection in results:
        joints=detection["pred_joints_img"]
   
        lenPoints=len(joints)
        person_msg = Person()
        person_msg.bodyParts = [None]*lenPoints
        for idx in range(lenPoints):
            person_msg.bodyParts[idx] = Bodypart()
        for idx in range(lenPoints):
            person_msg.bodyParts[idx].score = 0.8
            person_msg.bodyParts[idx].pixel.x = joints[idx,0]
            person_msg.bodyParts[idx].pixel.y = joints[idx,1]
            person_msg.bodyParts[idx].point.x =joints[idx,0]/200-3
            person_msg.bodyParts[idx].point.y =joints[idx,2]/200
            person_msg.bodyParts[idx].point.z = -joints[idx,1]/200+3.2
    
        msg.persons.append(person_msg)
    publisher.publish(msg)

def myhook():
    rospy.loginfo("Shutdown")
    rospy.signal_shutdown("Darum")
 
if __name__ == '__main__':
    global args 
    global body_bbox_detector
    global body_mocap

    args = DemoOptions().parse()

    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    assert torch.cuda.is_available(), "Current version only supports GPU"

    # Set bbox detector
    body_bbox_detector = BodyPoseEstimator()

    # Set mocap regressor
    use_smplx = args.use_smplx
    checkpoint_path = args.checkpoint_body_smplx if use_smplx else args.checkpoint_body_smpl
    print("use_smplx", use_smplx)
    body_mocap = BodyMocap(checkpoint_path, args.smpl_dir, device, use_smplx)
    
    rospy.init_node('spin', anonymous=True)
    rospy.Subscriber('image', Image, run_body_mocap)
    publisher = rospy.Publisher('personsJS', Persons, queue_size=2)
    rospy.on_shutdown(myhook)
    rospy.spin()