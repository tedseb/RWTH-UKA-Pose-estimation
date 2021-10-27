#!/usr/bin/python3
import numpy as np
from sensor_msgs.msg import Image
from backend.msg import Images

import time
import rospy
import message_filters

class cam_syn:
    def __init__(self):
        img_sub = message_filters.Subscriber('image', Image)
        img1_sub  =message_filters.Subscriber('image1', Image)
        ts = message_filters.ApproximateTimeSynchronizer([img_sub, img1_sub], 100, 0.03, allow_headerless=True)
        ts.registerCallback(self.addImages)
        self.publish_syn_imgs = rospy.Publisher('synchronized_images', Images, queue_size=2)

    def addImages(self, img1_msg, img2_msg):  
        image_msgs = Images()
        image_msgs.number = 2
        image_msgs.img1 = img1_msg
        image_msgs.img2 = img2_msg
        self.publish_syn_imgs.publish(image_msgs)

if __name__ == '__main__':  
    rospy.init_node('cam_syn', anonymous=True)
    cam_syn = cam_syn() # Later on, we can choose a specific detector. We have to write a new class for each detector
    rospy.spin()
