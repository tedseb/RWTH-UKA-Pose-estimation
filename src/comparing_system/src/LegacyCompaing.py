"""
This file contains some functions that have been taken from Tamers code and ajdusted a little bit.
They work spcicifically with the lagecy squat exercise definition.
This file and the code within will be discontinued soon and stay here for now as a fallback solution
"""
import math
import threading
from collections import deque

import redis
import rospy as rp
import yaml
#Old imports 
from backend.msg import Persons
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from src.joint_adapters.spin import *

last_30_poses = deque()
lastPose = {}
angles = {}
alpha = 20 # Beschreibt eine Art Schmidt-Trigger

state = 0
reps = 0

def compare_legacy(spot_info_dict: dict, past_joints_with_timestamp_list: list, joints_with_timestamp: list, future_joints_with_timestamp_list: dict):
    global lastPose
    current_exercise = spot_info_dict['exercise']
    
    joints = joints_with_timestamp['joints']
    timestamp = joints_with_timestamp['timestamp']

    pose = {}

    for index in ownpose_used:
        point = Point()
        # This code currently swaps Y and Z axis, which is how Tamer did this. # TODO: Find defenitive solution to this
        point.x = joints[index]['point']['x']
        point.y = joints[index]['point']['z']
        point.z = joints[index]['point']['y']
        pose[joint_labels[index]] = point

    # This corresponds to Tamers "save" method
    last_30_poses.append(pose)
    lastPose = pose
    if (len(last_30_poses) >= 30):
        last_30_poses.popleft()

    for angle, points in angle_joints_mapping.items():
        angles[angle] = calculateAngle(points)

    updated_repetitions = count(current_exercise)

    # We define the center of the body as the pelvis
    center_of_body = pose[center_of_body_label]

    return updated_repetitions, center_of_body


def count(current_exercise):
    """
    Check if the state has changed. If so, possibly increment repetitions and return them. Otherwise return None
    """
    
    # TODO: Get these global Variables into a class
    global state
    global reps

    if state < len(current_exercise['stages']) - 1 and checkforstate(angles, current_exercise, state + 1):
        state += 1
        rp.logerr("state" + str(state) + "reps" + str(reps))

    if (state >= len(current_exercise['stages']) - 1):
        if (checkforstate(angles, current_exercise, 0)):
            state = 0
            reps += 1
            rp.loginfo("Legacy Comparing: Reps: " + str(reps))
            return True
            

def checkforstate(angles, exercise, state):
    stage = exercise["stages"][state]["angles"]
    _pass = True
    for key in stage.keys():
        _pass = _pass and (angles[key] >= stage[key] - alpha and angles[key] <= stage[key] + alpha)
    return _pass

def calculateAngle(array):
    # Tamer used this function to calculate the angle between left hip, l knee and big toe
    if len(array) == 2:
        newPoint = Point(lastPose[array[1]].x, lastPose[array[1]].y - 1, lastPose[array[1]].z)
        return threepointangle(lastPose[array[0]], lastPose[array[1]], newPoint)
    elif len(array) == 3:
        return threepointangle(lastPose[array[0]], lastPose[array[1]], lastPose[array[2]])
    elif len(array) == 6:
        return (threepointangle(lastPose[array[0]], lastPose[array[1]], lastPose[array[2]]) + threepointangle(lastPose[array[3]], lastPose[array[4]], lastPose[array[5]])) / 2
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
