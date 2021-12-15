#!/usr/bin/python3
# -*- coding: utf-8 -*-


import rospy as rp
import argparse
import logy


try:
    from motion_analysis.src.Worker import *
    from motion_analysis.src.InterCom import *
    from motion_analysis.src.DataUtils import *
    from motion_analysis.src.algorithm.FeatureExtraction import *
    from motion_analysis.src.algorithm.AlgoUtils import *
except ImportError:
    from src.Worker import *
    from src.InterCom import *
    from src.DataUtils import *
    from src.algorithm.FeatureExtraction import *
    from src.algorithm.AlgoUtils import *


from backend.msg import Person
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Vector3


class Visualizer():
    def __init__(self,
    feature_extractor_class: PoseDefinitionAdapter = MetrabsPoseDefinitionAdapter):
        # define a publisher to publish the 3D skeleton of multiple people
        self.input_pub = rp.Publisher('motion_analysis_input_markers', MarkerArray, queue_size=100)
        self.reference_pub = rp.Publisher('motion_analysis_reference_markers', MarkerArray, queue_size=100)
        self.markerid = 0
        self.feature_extractor = feature_extractor_class()

        # define a subscriber to retrive tracked bodies
        rp.Subscriber('motion_analysis_input', Person, self.user_callback)
        rp.Subscriber('motion_analysis_reference_prediction', Person, self.reference_callback)

    def user_callback(self, data):
        return self.frame_callback(data, self.input_pub)

    def reference_callback(self, data):
        return self.frame_callback(data, self.reference_pub)

    # @logy.trace_time("visualizer:frame_callback", period=100) # This creates an error at the moment.
    def frame_callback(self, data, publisher):
        idx = 0
        marker_array = MarkerArray()

        for bodypart in data.bodyParts:
            if bodypart.score < 0.1:
                continue
            m = Marker()
            m.header.stamp = data.header.stamp
            m.header.frame_id = data.header.frame_id
            m.id = idx
            m.ns = ''
            m.color = ColorRGBA(0.98, 0.30, 0.30, 1.00)
            m.scale = Vector3(0.03,0.03,0.03)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = bodypart.point.x, (bodypart.point.y), bodypart.point.z
            m.type = 2
            m.action = 0
            m.lifetime = rp.Duration(0.2) 
            marker_array.markers.append(m)
            idx+=1

        for pair in self.feature_extractor.joint_connections:
            m = Marker()
            m.header = Header(self.markerid, rp.Time.now(), "map")
            self.markerid += 1
            m.id = idx
            idx+=1
            m.ns = ''
            m.color = ColorRGBA(0.98, 0.30, 0.30, 1.00)
            m.scale = Vector3(0.01,0.01,0.01)
            m.points = [data.bodyParts[pair[0]].point, data.bodyParts[pair[1]].point]
            m.type = 4
            m.action = 0
            m.lifetime = rp.Duration(0.5) 
            marker_array.markers.append(m)
            
        publisher.publish(marker_array)


if __name__ == '__main__':
    rp.init_node('Motion_Analysis_Visualizer', anonymous=False)

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
    
    if args.ai == 'spin':
        pose_definition_adapter_class = SpinPoseDefinitionAdapter
    elif args.ai == 'metrabs':
        pose_definition_adapter_class = MetrabsPoseDefinitionAdapter
    else:
        rp.logerr("Could not find a suitable PoseDefinition Adapter for ai argument: <" + str(args.ai) + ">")


    visualization = Visualizer(pose_definition_adapter_class)
    rp.spin()