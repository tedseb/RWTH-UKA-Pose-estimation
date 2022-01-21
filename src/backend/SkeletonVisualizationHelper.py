#!/usr/bin/python3
# -*- coding: utf-8 -*-
import math
import rospy
from backend.msg import Persons, ChannelInfo
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
import argparse
import sys
import logy
from multiprocessing import Lock
import math

ROTATE_DIM_X = -25
ROTATE_DIM_Y = 0
ROTATE_DIM_Z = 0

metrabs_pose = [
    (1,4),(1,0),(2,5),(2,0),(3,6),(3,0),(4,7),(5,8),(6,9),(7,10),(8,11),(9,12),(12,13),(12,14),(12,15),(13,16),(14,17),(16,18),(17,19),
    (18,20),(19,21),(20,22),(21,23)
]

spinvis_pose = [
    (2, 3), (3, 4), (5,6),(6,7),(27, 9),(9,12),(27,28), (27, 10), (10, 11), (12,13),(9,10), (28, 12), (28, 13) , (13,14), (14, 21), (21, 20),(21, 19),(20,19),(11,24),(24,22),(22,23),(23,24),(5,28),(2,27),(5,2),
    (42,17),(42,18),(42,0),(0,15),(0,16),(15,16),(17,43),(18,43),(1,37),(37,43),(41,37),(41,39)
]

def rotate_dim_x(coordinates, degree):
    radians = math.radians(degree)
    rotated_coordinates = [0., 0., 0.]
    rotated_coordinates[0] = coordinates[0]
    rotated_coordinates[1] = coordinates[1] * math.cos(radians) - coordinates[2] * math.sin(radians)
    rotated_coordinates[2] = coordinates[1] * math.sin(radians) + coordinates[2] * math.cos(radians)
    return rotated_coordinates

def rotate_dim_y(coordinates, degree):
    radians = math.radians(degree)
    rotated_coordinates = [0., 0., 0.]
    rotated_coordinates[1] = coordinates[1]
    rotated_coordinates[2] = coordinates[2] * math.cos(radians) - coordinates[0] * math.sin(radians)
    rotated_coordinates[0] = coordinates[2] * math.sin(radians) + coordinates[0] * math.cos(radians)
    return rotated_coordinates

def rotate_dim_z(coordinates, degree):
    radians = math.radians(degree)
    rotated_coordinates = [0., 0., 0.]
    rotated_coordinates[2] = coordinates[2]
    rotated_coordinates[0] = coordinates[0] * math.cos(radians) - coordinates[1] * math.sin(radians)
    rotated_coordinates[1] = coordinates[0] * math.sin(radians) + coordinates[1] * math.cos(radians)
    return rotated_coordinates

class RealtimeVisualization():
    def __init__(self, ai):
        rospy.Subscriber('personsJS', Persons, self.frame_callback)
        rospy.Subscriber('/channel_info', ChannelInfo, self.handle_new_channel)
        self.fused_skeleton_pub = rospy.Publisher('fused_skelleton', Persons, queue_size=100)
        self.visualization_skeleton_pubs = {}
        self._thread_lock = Lock()

        if ai == "metrabs":
            self.ownpose = metrabs_pose
            self.skeleton_scale = Vector3(0.1,0.1,0.1)
        elif ai == "spin":
            self.ownpose = spinvis_pose
            self.skeleton_scale = Vector3(0.03,0.03,0.03)

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


        # define a subscriber to retrive tracked bodies

    @logy.catch_ros
    def handle_new_channel(self, channel_info: ChannelInfo):
        with self._thread_lock:
            if channel_info.is_active:
                logy.debug(f"New Channel: {channel_info.channel_name}")
                if channel_info.cam_id not in self.visualization_skeleton_pubs:
                    pub = rospy.Publisher(f'/visualization/skeleton_{channel_info.channel_id}', MarkerArray, queue_size=100)
                    self.visualization_skeleton_pubs[channel_info.cam_id] = pub
            else:
                if channel_info.cam_id in self.visualization_skeleton_pubs:
                    self.visualization_skeleton_pubs[channel_info.cam_id].unregister()
                    del self.visualization_skeleton_pubs[channel_info.cam_id]


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
        marker.header.frame_id = 'dev0'
        marker.lifetime = rospy.Duration(1)  # 1 second
        return marker

    def translation(self, persons_data):
        dim_height = 2
        min_height = 9999
        persons : Persons = persons_data
        for person in persons:
            move_x = person.bodyParts[0].point.x
            move_y = person.bodyParts[0].point.y
            move_z = person.bodyParts[0].point.z
            for bodypart in person.bodyParts:
                coordinates = [bodypart.point.x, bodypart.point.y, bodypart.point.z]
                # Move to Origin
                coordinates[0] -= move_x
                coordinates[1] -= move_y
                coordinates[2] -= move_z
                # Rotate
                coordinates = rotate_dim_x(coordinates, ROTATE_DIM_X)
                coordinates = rotate_dim_y(coordinates, ROTATE_DIM_Y)
                coordinates = rotate_dim_z(coordinates, ROTATE_DIM_Z)

                if coordinates[dim_height] < min_height:
                    min_height = coordinates[dim_height]

                bodypart.point = Point(coordinates[0], coordinates[1], coordinates[2])

            for bodypart in person.bodyParts:
                coordinates = [bodypart.point.x, bodypart.point.y, bodypart.point.z]
                # Move up
                coordinates[dim_height] -= min_height
                bodypart.point = Point(coordinates[0], coordinates[1], coordinates[2])
        return persons

    @logy.catch_ros
    def frame_callback(self, data):
        '''
        This function will be called everytime whenever a message is received by the subscriber
        '''
        person_counter = 0
        idx = 0
        camera_id = int(data.header.frame_id[3:])
        marker_array = MarkerArray()
        persons = self.translation(data.persons)

        for person in persons:
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
                m.scale = self.skeleton_scale
                m.pose.position.x, m.pose.position.y, m.pose.position.z = bodypart.point.x, bodypart.point.y, bodypart.point.z
                m.type = 2
                m.action = 0
                m.lifetime = rospy.Duration(0.2)
                marker_array.markers.append(m)

            for pair in self.ownpose:
                if (person.bodyParts[pair[0]].score < 0.01) or (person.bodyParts[pair[1]].score < 0.01):
                    continue
                m = Marker()
                m.header.stamp = data.header.stamp
                m.header.frame_id = data.header.frame_id
                m.id = idx
                idx+=1
                m.ns = ''
                m.color = ColorRGBA(0.98, 0.30, 0.30, 1.00)
                m.scale = Vector3(0.01,0.01,0.01)
                m.points = [person.bodyParts[pair[0]].point, person.bodyParts[pair[1]].point]
                m.type = 4
                m.action = 0
                m.lifetime = rospy.Duration(0.2)
                marker_array.markers.append(m)

        # publish the markers
        #self.visualization_skeleton_pub.publish(marker_array)
        with self._thread_lock:
            if camera_id in self.visualization_skeleton_pubs:
                self.visualization_skeleton_pubs[camera_id].publish(marker_array)
            self.fused_skeleton_pub.publish(data)  # TODO: Fusion and normalization is missing here


if __name__ == '__main__':
    # define some constants
    logy.basic_config(logy.DEBUG, module_name="VIS")
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="Verbose mode", action="store_true") # TODO: @sm This does not do anything now - is it even necessary?
    parser.add_argument("-a", "--ai", help="Name of AI to work with", type=str, default='metrabs')
    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]
    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()

    # initialize ros node
    rospy.init_node('visualizer_node', anonymous=False)

    # read the parameters from ROS launch file
    id_text_offset = 0
    skeleton_line_width = 0.1

    # instantiate the RealtimeVisualization class
    visualization = RealtimeVisualization(args.ai)

    rospy.spin()
