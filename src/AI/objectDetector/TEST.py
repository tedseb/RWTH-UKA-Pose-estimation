#!/usr/bin/python3
import numpy as np
import cv2
import pafy
import matplotlib.pyplot as plt
from matplotlib import cm
from PIL import Image as ImagePil
from sensor_msgs.msg import Image
from backend.msg import Bboxes
from backend.msg import LabelsCameraID
from scheduler import getBoxesInStation
import time
import rospy


class ObjectDetectionPipeline:
    def __init__(self):
        rospy.Subscriber('labels', LabelsCameraID, self.run_objectdetector)
        self.spin() #Dont write any line of code after self.spin!

    def spin(self):
        '''
        We enter in a loop and wait for exit whenever `Ctrl + C` is pressed
        '''
        rospy.spin()

    def run_objectdetector(self, msg):  
        '''
        This is a callback function which receives the message from the subscriber. 
        The message contains an image from the camera, this is reshaped, 
        sent to the function obj_detectYolo() and then the result (BBOX) is published.
        '''
        print("Labels", msg.data)

                

if __name__ == '__main__':  
    rospy.init_node('objectNodeYOLO', anonymous=True)
    # instantiate the Comparator class
    obj_detect = ObjectDetectionPipeline()                  #Later on, we can choose a specific detector. We have to write a new class for each detector
