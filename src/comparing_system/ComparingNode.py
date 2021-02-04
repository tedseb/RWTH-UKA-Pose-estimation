#!/usr/bin/python3
# -*- coding: utf-8 -*-
import math
import rospy
from pose_estimation.msg import Persons, commands
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

class elComparador():
    def __init__(self, publish_topic, subscribe_topic, skeleton_frame):
        self.publish_topic = publish_topic
        self.skeleton_frame = skeleton_frame

        # define a publisher to publish the 3D skeleton of multiple people
        self.command_pub = rospy.Publisher(self.publish_topic, String, queue_size=100)

        # define a subscriber to retrive tracked bodies
        rospy.Subscriber(subscribe_topic, Persons, self.frame_callback)


    def spin(self):
        '''
        We enter in a loop and wait for exit whenever `Ctrl + C` is pressed
        '''
        rospy.spin()


    def frame_callback(self, data):
        '''
        This function will be called everytime whenever a message is received by the subscriber
        '''
        idx = 0
        print("vis: ", data)
        msg_array = list()
        for person in data.persons:
            idx+=1 
            for bodypart in person.bodyParts:
                print("--------------------------------------")
                print("persons: ", bodypart)   
                #Use for your calculation for example this: bodypart.point.x, bodypart.point.y , bodypart.point.z
                msg = commands()
                msg.id = idx                                            #Not sure if self.frame_callback or only id is the correct choice for the skelleton definition
                msg.data = "Kopf hoch kleines"
                msg_array.append(msg)
            

        # publish the markers
        self.command_pub.publish(msg_array)
        #print("inhalt :", msg_array)


if __name__ == '__main__':
    # define some constants
    publish_topic_2user = 'commands2user'

    # initialize ros node
    rospy.init_node('comparing_system_node', anonymous=False)

    # read the parameters from ROS launch file
    subscribe_topic = 'fused_skelleton'
    skeleton_frame = '1' #fetch from parameter server. Currently not in use

    # instantiate the elComparador class
    elComparador_de_la_noche = elComparador(publish_topic_2user, subscribe_topic, skeleton_frame)
    elComparador_de_la_noche.spin()