#!/usr/bin/python3
# -*- coding: utf-8 -*-
import math
import rospy
from pose_estimation.msg import Persons
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
from pose2d.tf_pose import common 

ownpose = [
    (1, 2), (1, 3), (2, 4), (3,5), (4, 6), (7, 8), (7, 9), (9, 11), (8, 10) , (10,12), (1, 7), (2, 8)
    ]

kinect_pose = [(2,1), (1,0), (2,3), (3,26), (26,27), (2,4), (4,5), (5,6), (6,7), (7,8), (8,9), (7,10), \
    (0,18), (18,19), (19,20), (20,21), (27,28), (28,29), (2,11), (11,12), (12,13), (13,14), (14,15), \
    (15,16), (14,17), (0,22), (22,23), (23,24), (24,25), (27,30), (30,31)]

joint_num=32

class RealtimeVisualization():
    def __init__(self, pub_topic, sub_topic, frame_id):
        self.pub_topic = pub_topic
        self.frame_id = frame_id



        # define a publisher to publish the 3D skeleton of multiple people
        self.pub = rospy.Publisher(self.pub_topic, MarkerArray, queue_size=100)
        # define a subscriber to retrive tracked bodies
        rospy.Subscriber(frame_topic, Persons, self.frame_callback)


    def spin(self):
        '''
        We enter in a loop and wait for exit whenever `Ctrl + C` is pressed
        '''
        rospy.spin()


    def create_marker(self, index, color, marker_type, size, time):
        '''
        Function to create a visualization marker which is used inside RViz
        '''
        marker = Marker()
        marker.id = index
        marker.ns = self.ns
        marker.color = color
        marker.action = Marker.ADD
        #marker.type = marker_type
        marker.scale = Vector3(size, size, size)
        marker.header.stamp = time
        marker.header.frame_id = self.skeleton_frame
        marker.lifetime = rospy.Duration(1)  # 1 second
        return marker


    def frame_callback(self, data):
        '''
        This function will be called everytime whenever a message is received by the subscriber
        '''
        marker_counter = 0
        person_counter = 0
        idx = 0
        marker_array = MarkerArray()

        for person in data.persons:
            marker_color = self.colors[person_counter % len(self.colors)]
            

            for bodypart in person.bodyParts:
                if bodypart.score < 0.1:
                    continue
                
                m = Marker()
                m.header.stamp = data.header.stamp
                m.header.frame_id = data.header.frame_id
                m.id = idx
                idx+=1
                m.ns = ''
                m.color = ColorRGBA(0.98, 0.30, 0.30, 1.00)
                m.scale = Vector3(0.1,0.1,0.1)
                m.pose.position.x, m.pose.position.y, m.pose.position.z = bodypart.point.x, bodypart.point.y, bodypart.point.z
                m.type = 2
                m.action = 0
                m.lifetime = rospy.Duration(0.2) 
                marker_array.markers.append(m)
                
            for pair in kinect_pose:
                    if (person.bodyParts[pair[0]].score < 0.01) or (person.bodyParts[pair[1]].score < 0.01):
                        #print(person.bodyParts[pair[0]].score)
                        #print(person.bodyParts[pair[1]].score)
                        continue
                    m = Marker()
                    m.header.stamp = data.header.stamp
                    m.header.frame_id = data.header.frame_id
                    m.id = idx
                    idx+=1
                    m.ns = ''
                    m.color = ColorRGBA(0.98, 0.30, 0.30, 1.00)
                    m.scale = Vector3(0.05,0.1,0.1)
                    m.points = [person.bodyParts[pair[0]].point, person.bodyParts[pair[1]].point]
                    m.type = 4
                    m.action = 0
                    m.lifetime = rospy.Duration(0.5) 
                    marker_array.markers.append(m)

        # publish the markers
        self.pub.publish(marker_array)


if __name__ == '__main__':
    # define some constants
    sub_topic = 'kinect_persons'

    # initialize ros node
    rospy.init_node('visualizer_node', anonymous=False)

    # read the parameters from ROS launch file
    frame_topic = 'kinect_marker'
    frame_id = 'dev0'

    # instantiate the RealtimeVisualization class
    visualization = RealtimeVisualization(sub_topic, frame_topic, frame_id)
    visualization.spin()
