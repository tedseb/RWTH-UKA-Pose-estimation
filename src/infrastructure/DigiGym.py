#!/usr/bin/python3 
import rospy
import cv2
from sensor_msgs.msg import Image
from backend.msg import Bboxes
import numpy as np
import pafy
import yaml
import os
# from utils.imutils import crop_bboxInfo, process_image_bbox, process_image_keypoints, bbox_from_keypoints ToDo: Do cropping here. 



class run_spin():
    def __init__(self,station_ID,incSpace ):
        
        self.incSpace=incSpace
        self.station_ID=station_ID
        # define a publisher to publish the 3D skeleton of multiple people
        self.publisher_crop = rospy.Publisher('cropImage', Image, queue_size=2)   

        # define a subscriber to retrive tracked bodies
        rospy.Subscriber('bboxes', Bboxes, self.callback_station)
        rospy.Subscriber('image', Image, self.callback_setImage)
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
        self.sensor_name = msg.header.frame_id

    def callback_station(self, body_bbox_list_station):
        '''
        This function will be called everytime whenever a message is received by the subscriber
        '''
        print("Detection: ", body_bbox_list_station.data)
        bbox=np.array(body_bbox_list_station.data).reshape(-1,4)[0]
        print("Detection_reshaped: ", bbox)
        station_end_x= bbox[2]+bbox[0]
        station_end_y= bbox[3]+bbox[1]
        x1=bbox[0]
        y1=bbox[1]
        img_draw=self.img_original_bgr.copy()
        start_point = (int(x1),int(y1)) 
        end_point =  (int(station_end_x), int(station_end_y)) 

        x1_ =int(x1-(self.incSpace*bbox[2]))
        y1_ =int(y1-(self.incSpace*bbox[3]))
        if x1_<0:
            x1_=0
        if y1_<0:
            y1_=0
        start_point2 = (x1_,y1_)   
            
        x2_ =int(station_end_x+(self.incSpace*bbox[2]))
        y2_ =int(station_end_y+(self.incSpace*bbox[3]))        
        if x2_>self.msg_image.width:                       # note (h,w,3) 
            x2_=self.msg_image.width
        if y2_>self.msg_image.height:
            y2_=self.msg_image.height        
        end_point2 =  (x2_, y2_) 

        color1 = (255, 0, 0) 
        color2 = (0, 255, 0) 
        thickness = 3

    
        img_draw = cv2.rectangle(img_draw, start_point, end_point, color1, thickness)
        img_draw = cv2.rectangle(img_draw, start_point2, end_point2, color2, thickness)
        
            
        if os.path.exists('/home/trainerai/trainerai-core/src/infrastructure/stations/'+ str(self.sensor_name)+ '.yaml'):
            with open(r'/home/trainerai/trainerai-core/src/infrastructure/stations/'+ str(self.sensor_name)+ '.yaml') as file:
                station_dic = yaml.load(file,Loader=yaml.Loader)
            print("Content: ",station_dic) 
        else:
            station_dic={}
            print("Content: ",station_dic) 

        #station_dic[(self.station_ID)] = [(bbox[0]-(self.incSpace*bbox[2])), (bbox[1]-(self.incSpace*bbox[3])), ((1+self.incSpace)*bbox[2]), ((1+self.incSpace)*bbox[3])]
        station_dic[(self.station_ID)] = [start_point2[0], start_point2[1], end_point2[0], end_point2[1]]
        with open('/home/trainerai/trainerai-core/src/infrastructure/stations/'+ str(self.sensor_name)+'.yaml', "w+") as file:  # Safely open the file
            documents = yaml.dump(station_dic, file)
        cv2.imwrite('/home/trainerai/trainerai-core/src/infrastructure/stations/' + str(self.sensor_name) + '_station_ID_'+ str(self.station_ID)+'.png',img_draw)
            

if __name__ == '__main__':
    #global station_ID
    station_ID=2
    incSpace=0.25

    rospy.init_node('spin', anonymous=True)
    # instantiate the Comparator class
    run_spin_obj = run_spin(station_ID,incSpace)
