"""
This file contains code that deals with the  manual extraction of low-level features from skelletons.
At the time of writing, such features are explicitly angles. Other features might follow in the future.

TODO: Use msgpack at Shawans end do extract features there already? https://pypi.org/project/msgpack-numpy/
"""
from abc import ABC, abstractmethod
import math
import numpy as np
from geometry_msgs.msg import Point, Vector3
import rospy as rp
from itertools import combinations

from src.joint_adapters.spin import *

class FeatureExtractorException(Exception):
    pass

class UnknownAngleException(FeatureExtractorException):
    pass


# class FeatureExtractor(ABC):
#     @abstractmethod
#     def __init__():
#         pass

#     @abstractmethod
#     def extract_features(joints: list):
#         """
#         This method takes a list of joints and transforms them to a list of features.
#         """
#         pass


# class SpinFeatureExtractor(ABC):
#     def __init__():
#         raise NotImplementedError

#     def extract_features(joints: list):
#         raise NotImplementedError


def calculateAngle(inner_joint, outer_joints, pose):
    outer_joints = tuple(outer_joints)
    return threepointangle(pose[inner_joint], pose[outer_joints[0]], pose[outer_joints[1]])

def angle3d(a, b):
    x = dot_product(a, b) / (length_of_vector(a) * length_of_vector(b))
    return math.acos(x) * 180 / math.pi

def threepointangle(inner_joint, outer_joint_1, outer_joint_2):
    ba = create_vector_from_two_points(inner_joint, outer_joint_1)
    bc = create_vector_from_two_points(inner_joint, outer_joint_2)
    return angle3d(ba, bc)

def create_vector_from_two_points(a, b):
    return Vector3(b.x - a.x, b.y - a.y, b.z - a.z)

def dot_product(a, b):
    return a.x * b.x + a.y * b.y + a.z * b.z

def length_of_vector(x):
    return math.sqrt(math.pow(x.x, 2) + math.pow(x.y, 2) + math.pow(x.z, 2))

def extract_boundaries(exercise_data, angles_of_interest=set(), distances_of_interest=set()):
    """
    Extracts the boundaries of angles (and in the future possibly distances).
    """
    angles_low = {}
    angles_high = {}

    exceptions = dict()

    for step in exercise_data['recording']:
        skeleton = step[1]

        pose = {}
        angles = {}

        # TODO: Simplify this
        for joint_names in angles_of_interest:
            for joint_name in joint_names: 
                point = Point()
                # This code currently swaps Y and Z axis, which is how Tamer did this. # TODO: Find defenitive solution to this
                point.x = skeleton[joint_name]['x']
                point.y = skeleton[joint_name]['z']
                point.z = skeleton[joint_name]['y']
                pose[joint_name] = point

        for joint_names in angles_of_interest:
            try:
                inner_joint, outer_joints = find_inner_and_outer_joints(joint_names)
            except UnknownAngleException as e:
                exceptions['UnknownAngleException'] = e
                continue
            except FeatureExtractorException as e:
                exceptions['FeatureExtractorException'] = e

            angle = calculateAngle(inner_joint, outer_joints, pose)

            if angle < angles_low.get(joint_names, math.inf):
                angles_low[joint_names] = angle

            if angle > angles_high.get(joint_names, -math.inf):
                angles_high[joint_names] = angle
    
    boundaries = {"angles": {"angles_low": angles_low, "angles_high": angles_high}, "distances":{}}

    if exceptions:
        rp.logerr("Errors occured while parsing the provided exercise:" + str(exceptions))

    return boundaries

def find_inner_and_outer_joints(joint_names):
    if not len(joint_names) == 3:
        raise FeatureExtractorException("Can not calculate angle between other than 3 points.")
    
    joint_connection_combinations = combinations(joint_names, 2)

    # We must find two connections between our three joints, such that there is a usefull angle between them
    combinations_found = 0

    for joint_connection_combination in joint_connection_combinations:
        if frozenset(joint_connection_combination) not in joint_connections_labels:
            outer_joints = joint_connection_combination
            inner_joint, *_ = joint_names.difference(joint_connection_combination)
        else:
            combinations_found += 1

    if combinations_found == 2:
        return inner_joint, outer_joints
    else:
        raise UnknownAngleException("Specified angle between joints that are not connected")
    