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

from src.joint_adapters.spin import *


class FeatureExtractor(ABC):
    @abstractmethod
    def __init__():
        pass

    @abstractmethod
    def extract_features(joints: list):
        """
        This method takes a list of joints and transforms them to a list of features.
        """
        pass


class SpinFeatureExtractor(ABC):
    def __init__():
        raise NotImplementedError

    def extract_features(joints: list):
        angles = dict()

        for index in ownpose_used:
            point = Point()
            # This code currently swaps Y and Z axis, which is how Tamer did this. # TODO: Find defenitive solution to this
            point.x = joints[index]['point']['x']
            point.y = joints[index]['point']['z']
            point.z = joints[index]['point']['y']
            pose[joint_labels[index]] = point

        for angle, points in angle_joints_mapping.items():
            angles[angle] = calculateAngle(points)

        return angles


def calculateAngle(array, pose):
    # Tamer used this function to calculate the angle between left hip, l knee and big toe
    if len(array) == 2:
        newPoint = Point(pose[array[1]].x, pose[array[1]].y - 1, pose[array[1]].z)
        return threepointangle(pose[array[0]], pose[array[1]], newPoint)
    elif len(array) == 3:
        return threepointangle(pose[array[0]], pose[array[1]], pose[array[2]])
    elif len(array) == 6:
        return (threepointangle(pose[array[0]], pose[array[1]], pose[array[2]]) + threepointangle(pose[array[3]], pose[array[4]], pose[array[5]])) / 2
    return 0


# a and b are the start and end point of a given vector, which is perpendicular to the plane
# p is a point of that plane
# x will be checked against that plane
# b_ = {x: b.x, y: a.y, z: b.z}; for a plane that is 
def plane3d (a, b, p, x):
    # TODO: Clean this up
    direction = create_vector_from_two_points(a, b)
    val = dot_product(Vector3(x.x - p.x, x.y - p.y, x.z - p.z), direction)
    return val


def angle3d(a, b):
    x = dot_product(a, b) / (length_of_vector(a) * length_of_vector(b))
    return math.acos(x) * 180 / math.pi


def threepointangle(a, b, c):
    ba = create_vector_from_two_points(b, a)
    bc = create_vector_from_two_points(b, c)
    return angle3d(ba, bc)


def create_vector_from_two_points(a, b):
    return Vector3(b.x - a.x, b.y - a.y, b.z - a.z)


def dot_product(a, b):
    return a.x * b.x + a.y * b.y + a.z * b.z


def length_of_vector(x):
    return math.sqrt(math.pow(x.x, 2) + math.pow(x.y, 2) + math.pow(x.z, 2))


def extract_angle_boundaries(exercise_data):
    boundaries = dict()
    angles_of_interest = set()
    for stage in exercise_data['stages']:
        for rule_joints in stage['angles']:
            for angle, points in angle_joints_mapping.items():
                if set(rule_joints) == set(points):
                    angles_of_interest.update(angle)

    for step in exercise_data['recording']:
        skeleton = step[1]

        pose = {}
        angles = {}

        for angle, points in angle_joints_mapping.items():
            if angle in angles_of_interest:
                for point_label in points: 
                    point = Point()
                    # This code currently swaps Y and Z axis, which is how Tamer did this. # TODO: Find defenitive solution to this
                    point.x = skeleton[point_label]['x']
                    point.y = skeleton[point_label]['z']
                    point.z = skeleton[point_label]['y']
                    pose[point_label] = point
                
            angles[angle] = calculateAngle(points, pose)

    return boundaries