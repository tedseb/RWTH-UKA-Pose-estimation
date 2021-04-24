#!/usr/bin/python3
import numpy as np
from sensor_msgs.msg import Image

import time
import rospy
import message_filters

class cam_syn:
    def __init__(self):
        img_sub = message_filters.Subscriber('image', Image)
        img1_sub  =message_filters.Subscriber('image1', Image)
        ts = message_filters.ApproximateTimeSynchronizer([img_sub, img1_sub], 10, 0.04, allow_headerless=True)
        ts.registerCallback(self.run_objectdetector)
        publish_syn_imgs = rospy.Publisher('syn_image', Image, queue_size=2)
        self.spin() 
    def spin(self):
            '''
            We enter in a loop and wait for exit whenever `Ctrl + C` is pressed
            '''
            rospy.spin()

    def run_objectdetector(self, img_msg,img1_msg):  
        syn_imgs=[]

        shape = img_msg.height, img_msg.width, 3                            #(480, 640, 3) --> (y,x,3)
        img = np.frombuffer(img_msg.data, dtype=np.uint8)
        img_original_bgr_0 = img.reshape(shape)
        self.frame_id_0=int(img_msg.header.frame_id[3:])
        imgs.append(img_original_bgr_1)

        shape = img1_msg.height, img1_msg.width, 3                            #(480, 640, 3) --> (y,x,3)
        img = np.frombuffer(img1_msg.data, dtype=np.uint8)
        img_original_bgr_1 = img.reshape(shape)
        self.frame_id_1=int(img1_msg.header.frame_id[3:])
        imgs.append(img_original_bgr_1)
        
        syn_imgs=cv2.vconcat(imgs)  
        msg_cropImage = Image()
        msg_cropImage.header.stamp = rospy.Time.now()
        msg_cropImage.header.frame_id =  str(frame_id_0) + ',' + str(frame_id_1)
        msg_cropImage.encoding = "bgr8"
        msg_cropImage.data = np.array(syn_imgs, dtype=np.uint8).tostring()
        msg_cropImage.height, msg_cropImage.width = syn_imgs.shape[:-1]
        msg_cropImage.step = syn_imgs.shape[-1]*syn_imgs.shape[0]

        self.publish_syn_imgs.publish(msg_cropImage)

if __name__ == '__main__':  
    rospy.init_node('cam_syn', anonymous=True)
    # instantiate the Comparator class
    cam_syn = cam_syn()                  #Later on, we can choose a specific detector. We have to write a new class for each detector
