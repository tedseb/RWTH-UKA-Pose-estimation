#!/usr/bin/python3
# -*- coding: utf-8 -*-
import math
import rospy
from backend.msg import Person
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray

ownpose = [
    (2, 3), (3, 4), (5,6),(6,7), (27, 9),(9,12),(27,28), (27, 10), (10, 11), (12,13),(9,10), (28, 12), (28, 13) , (13,14), (14, 21), (21, 20),(21, 19),(20,19),(11,24),(24,22),(22,23),(23,24),(5,28),(2,27),(5,2),
    (42,17),(42,18),(42,0),(0,15),(0,16),(15,16),(17,43),(18,43),(1,37),(37,43),(41,37),(41,39)
    ]

class RealtimeVisualization():
    def __init__(self, ns, skeleton_frame, id_text_size, id_text_offset, skeleton_line_width):
        self.ns = ns
        self.skeleton_frame = skeleton_frame
        self.id_text_size = id_text_size
        self.id_text_offset = id_text_offset
        self.skeleton_line_width = skeleton_line_width

        # define a few colors we are going to use later on
        self.colors = [ColorRGBA(0.12, 0.63, 0.42, 1.00),
                       ColorRGBA(0.98, 0.30, 0.30, 1.00),
                       ColorRGBA(0.26, 0.09, 0.91, 1.00),
                       ColorRGBA(0.77, 0.44, 0.14, 1.00),
                       ColorRGBA(0.92, 0.73, 0.14, 1.00),
                       ColorRGBA(0.00, 0.61, 0.88, 1.00),
                       ColorRGBA(1.00, 0.65, 0.60, 1.00),
                       ColorRGBA(0.59, 0.00, 0.56, 1.00)]

        # define a publisher to publish the 3D skeleton of multiple people
        self.input_pub = rospy.Publisher('comparing_input_marker', MarkerArray, queue_size=100)
        self.reference_pub = rospy.Publisher('comparing_reference_prediction_marker', MarkerArray, queue_size=100)
        self.markerid = 0

        # define a subscriber to retrive tracked bodies
        rospy.Subscriber('comparing_input', Person, self.user_callback)
        rospy.Subscriber('comparing_reference_prediction', Person, self.reference_callback)


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

    def user_callback(self, data):
        return self.frame_callback(data, self.input_pub)

    def reference_callback(self, data):
        return self.frame_callback(data, self.reference_pub)

    def frame_callback(self, data, publisher):
        '''
        This function will be called everytime whenever a message is received by the subscriber
        '''
        marker_counter = 0
        person_counter = 0
        idx = 0

        marker_array = MarkerArray()

        person = data

        # marker_color = self.colors[person_counter % len(self.colors)]
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
            m.scale = Vector3(0.03,0.03,0.03)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = bodypart.point.x, (bodypart.point.y), bodypart.point.z
            m.type = 2
            m.action = 0
            m.lifetime = rospy.Duration(0.2) 
            marker_array.markers.append(m)

        for pair in ownpose:
            m = Marker()
            m.header = Header(self.markerid, rospy.Time.now(), "dev0")
            self.markerid += 1
            m.id = idx
            idx+=1
            m.ns = ''
            m.color = ColorRGBA(0.98, 0.30, 0.30, 1.00)
            m.scale = Vector3(0.01,0.01,0.01)
            m.points = [person.bodyParts[pair[0]].point, person.bodyParts[pair[1]].point]
            m.type = 4
            m.action = 0
            m.lifetime = rospy.Duration(0.5) 
            marker_array.markers.append(m)
            
        # publish the markers
        publisher.publish(marker_array)


if __name__ == '__main__':
    # define some constants
    ns = 'compviz'

    # initialize ros node
    rospy.init_node('compviz_node', anonymous=False)

    # read the parameters from ROS launch file
    skeleton_frame = 'dev0'
    id_text_size = 12
    id_text_offset = 0
    skeleton_line_width = 0.1

    # instantiate the RealtimeVisualization class
    visualization = RealtimeVisualization(ns, skeleton_frame, id_text_size, id_text_offset, skeleton_line_width)
    visualization.spin()
