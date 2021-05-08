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
        self.publish_syn_imgs = rospy.Publisher('syn_image', Images, queue_size=2)
        self.spin() 
    def spin(self):
            '''
            We enter in a loop and wait for exit whenever `Ctrl + C` is pressed
            '''
            rospy.spin()

    def addImages(self, img1_msg,img2_msg):  
        syn_imgs=[]
 
        
        image_msgs = Images()
        image_msgs.number = 2
        image_msgs.img1 = img1_msg
        image_msgs.img2 = img2_msg
        """
        imgs.append(img_original_bgr_1) 
        syn_imgs=cv2.vconcat(imgs)  
        msg_cropImage = Image()
        msg_cropImage.header.stamp = rospy.Time.now()
        msg_cropImage.header.frame_id =  str(frame_id_0) + ',' + str(frame_id_1)
        msg_cropImage.encoding = "bgr8"
        msg_cropImage.data = np.array(syn_imgs, dtype=np.uint8).tostring()
        msg_cropImage.height, msg_cropImage.width = syn_imgs.shape[:-1]
        msg_cropImage.step = syn_imgs.shape[-1]*syn_imgs.shape[0]
        """
        self.publish_syn_imgs.publish(image_msgs)

if __name__ == '__main__':  
    rospy.init_node('cam_syn', anonymous=True)
    # instantiate the Comparator class
    cam_syn = cam_syn()                  #Later on, we can choose a specific detector. We have to write a new class for each detector
