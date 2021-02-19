#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Shawans Imports
import math
import numpy as np
import rospy as rp

from backend.msg import Persons
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# Arturs Imports
import threading
import sqlite3
from importlib import import_module
from collections import deque
import yaml
from joint_adapters.spin_adapter import *

# Datafields from Tamer:

# TODO: What are those three?
last_30_poses = deque()
lastPose = {}
angles = {}
alpha = 20 # Beschreibt eine Art Schmidt-Trigger

# TODO: Auslagern
state = 0
states = dict()
corrections = dict()
reps = 0


# Artur's Datafields:
# TODO: Read the parameters from ROS launch file

current_exercise = None

JOINT_ADAPTER_FILE = 'joint_adapters.spin_adapter'


# TODO: Switch this for a ROS_TOPIC_USER_EXERCISE_STATES ROS_TOPIC
ROS_TOPIC_USER_EXERCISE_STATES = '/user_states'
ROS_TOPIC_USER_CORRECTIONS = '/user_corrections'

ROS_TOPIC_REPETITION_COUNTER = '/repcounter'  # Shawan topic choice: commands2user
ROS_TOPIC_CORRECTIONS = '/corrections'
ROS_TOPIC_WRONG_COORDINATES = '/wrongcoordinates'
ROS_TOPIC_SECONDS_SINCE_START = '/seconds_since_start'

ROS_TOPIC_CALLBACK = 'fused_skelleton' # Shawan's topic choice


import_module(JOINT_ADAPTER_FILE)


class Comparator():
    def __init__(self):
        # Define a publisher to publish the 3D skeleton of multiple people
        self.user_exercise_state_publisher = rp.publisher(ROS_TOPIC_USER_EXERCISE_STATES)


        self.repetition_publisher = rp.Publisher(ROS_TOPIC_REPETITION_COUNTER, String, queue_size=100)
        self.error_publisher = rp.Publisher(ROS_TOPIC_CORRECTIONS, String, queue_size=100)
        self.coordinates_publisher = rp.Publisher(ROS_TOPIC_WRONG_COORDINATES, String, queue_size=100)
        # Define a subscriber to retrive tracked bodies
        rp.Subscriber(ROS_TOPIC_CALLBACK, Persons, self.callback)

        rp.spin()

    def callback(self, msg):
        '''
        This function will be called everytime whenever a message is received by the subscriber
        '''
        global lastPose
        global current_exercise
        global ownpose_used


        pose = {}
        bodyParts = msg.persons[0].bodyParts # TODO: This currently uses only the first person, which we not sustainable, as we want to track multiple people

        try:
            current_exercise = yaml.safe_load(rp.get_param('exercise'))
        except KeyError as e:
            print("No exercise set under rostopic 'exercise', maybe the expert system is not running or ROS is not properly set up?")
            return

        if current_exercise['name'] not in states.keys():
            states[current_exercise['name']] = []
        
        if current_exercise['name'] not in corrections.keys():
            corrections[current_exercise['name']] = {1: "", 2: ""}

        for index in ownpose_used:
            point = Point()
            # This code currently swaps Y and Z axis, which is how Tamer did this. # TODO: Find defenitive solution to this
            point.x = bodyParts[index].point.x
            point.y = bodyParts[index].point.z
            point.z = bodyParts[index].point.y
            pose[joint_labels[index]] = point

        # This corresponds to Tamers "save" method
        last_30_poses.append(pose)
        lastPose = pose
        if (len(last_30_poses) >= 30):
            last_30_poses.popleft()
            # console.log(checkForStretch(last30)) # TODO: This is nodejs code from Tamer, find out what it does

        for angle, points in joints.items():
            angles[angle] = calculateAngle(points)
        
        # The following commented out code is not Python, but leftovers from Tamer for completeness
        # angles.leftLeg = threepointangle(pose.leftHip, pose.leftKnee, pose.leftAnkle);
        # angles.rightLeg = threepointangle(pose.rightHip, pose.rightKnee, pose.rightAnkle);
        # angles.leftArm = threepointangle(pose.leftShoulder, pose.leftElbow, pose.leftWrist);
        # angles.rightArm = threepointangle(pose.rightShoulder, pose.rightElbow, pose.rightWrist);
        # angles.upperBody = (threepointangle(pose.leftShoulder, pose.leftHip, pose.leftKnee) + threepointangle(pose.rightShoulder, pose.rightHip, pose.rightKnee)) / 2;
        # const bottomLeft = { x: pose.leftAnkle.x, y: pose.leftAnkle.y - 1, z: pose.leftAnkle.z };
        # const bottomRight = { x: pose.rightAnkle.x, y: pose.rightAnkle.y - 1, z: pose.rightAnkle.z };
        # angles.leftShin = threepointangle(pose.leftKnee, pose.leftAnkle, bottomLeft);
        # angles.rightShin = threepointangle(pose.rightKnee, pose.rightAnkle, bottomRight); *

        self.count()
        self.correct(angles, current_exercise, str(state))


        # This code was initially written by Shawan when he passed the code to me but does not fit what Tamer did
        # reply = list()
        # for idx, person in range(len(msg.persons)), msg.persons:  # TODO: Check if this works
        #     print("--------------------------------------")
        #     print("Persons: ")
        #     for bodypart in person.bodyParts:
        #         print(bodypart)
        #         # Use for your calculation for example this: bodypart.point.x, bodypart.point.y , bodypart.point.z
        #         msg = commands()
        #         msg.id = idx  # Not sure if self.frame_callback or only id is the correct choice for the skelleton definition
        #         msg.data = "test"
        #     reply.append(msg)
        # publish the markers
        # self.repetition_publisher.publish(reply)

        
    def correct(self, angles, exercise, state):
        messages = self.checkForCorrection(angles, exercise['stages'], state)
        if (messages == corrections[exercise['name']].get(state)):
            pass # TODO: Herausfinden was sich Tamer dabei gedacht hat
        else:
            corrections[exercise['name']][state] = messages
            if (len(messages) > 0):
                self.error_publisher.publish(messages)
                print(messages.encode('utf-8').strip())
    
    def checkForCorrection(self, angles, stages, state):
        """
        As of now, the default arguments to this function are: angles, squats, state
        """
        rules = stages[state]["rules"]
        corrections = ""
        for key in rules.keys():
            rule = rules[key][0]
            if rule == "max":
                if (angles[key] >= rules[key][1] + alpha):
                    corrections += rules[key][2] + ". "
                    # console.log(JSON.stringify(angle_points[k]))
                    self.coordinates_publisher.publish(str(joints[key]))
            elif rule == "min":
                if (angles[key] <= rules[key][1] - alpha):
                    corrections += rules[key][2] + ". "
                    # console.log(JSON.stringify(angle_points[k]))
                    self.coordinates_publisher.publish(str(joints[key]))
            elif rule == "behind":
                # a: 0, b: 1, p: 1, x: 2
                a = lastPose[joints[key][0]]
                b = lastPose[joints[key][1]]
                p = lastPose[joints[key][1]]
                x = lastPose[joints[key][2]]
                val_x = plane3d(a, b, p, x)
                val_a = plane3d(a, b, p, a)
                if (math.copysign(1, val_x) == math.copysign(1, val_a)):
                    corrections += rules[key][2] + ". "
                    self.coordinates_publisher.publish(str(joints[key]))
        return corrections

    def count(self):
        # TODO: Get these global Variables into a class
        global state
        global reps
        global states

        if state < 3 and checkforstate(angles, current_exercise, str(state + 1)):
            # states.squats.push(states + 1)
            state += 1
        
        if (state == 3):
            if (checkforstate(angles, current_exercise, str(1))):
                states['squats'].append(1)
                state = 1
                reps += 1
                # console.log(reps)
                self.repetition_publisher.publish(str(reps))
        # if (state == 0):
        #     if (checkforstate(angles, squats, 1)):
        #         states['squats'].append(1)
        #         state = 1
        
        # elif(state == 1):
        #     if (checkforstate(angles, squats, 2)):
        #         states['squats'].append(2)
        #         state = 2
        # elif(state == 2):
        #     if(checkforstate(angles, squats, 1)):
        #         states['squats'].append(1)
        #         state = 1
        #         reps += 1
        #         # console.log(reps)
        #         self.repetition_publisher.publish(commands(data=str(reps)))
        # if (checkforrep(states['squats'])):
        #     pass  # TODO: Again, why is this here?
        #     # reps = checkforrep

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


def checkforstate(angles, exercise, state):
    stage = exercise["stages"][state]["angles"]
    _pass = True
    for key in stage.keys():
        _pass = _pass and (angles[key] >= stage[key] - alpha and angles[key] <= stage[key] + alpha)
    return _pass

def checkforrep(array):
    # TODO: This code is quite ugly
    if array and (array[0] != 1 or array[len(array) - 1] != 1):
        return False
    for i in range(len(array)):
        if (array[i] - array[i - 1] > 1):
            return False
    return True

# a and b are the start and end point of a given vector, which is perpendicular to the plane
# p is a point of that plane
# x will be checked against that plane
# b_ = {x: b.x, y: a.y, z: b.z}; for a plane that is 
def plane3d (a, b, p, x):
    # TODO: Clean this up
    direction = create_vector_from_two_points(a, b)
    val = dot_product(Vector3(x.x - p.x, x.y - p.y, x.z - p.z), direction)
    return val

def checkForStretch(arr):
    # TODO: Clean this up
    lastAngles = {}
    currentAngles = {}
    differences = {}
    for key in joints.keys():
        differences[key] = []
    for element in arr:
        for angle, points in joints:
            currentAngles[angle] = calculateAngle(points)
        if (lastAngles):
            for key in lastAngles.keys():
                differences[key].append(currentAngles[key] - lastAngles[key])
        else:
            lastAngles = currentAngles
        
    sums = {}
    for angle, diff in differences:
        sum = 0
        for element in diff:
            sum += element
        sums[angle] = sum
    

    if (sums['leftLeg'] >= 0 or sums['rightLeg'] >= 0):
        return True
    else:
        return False

def angle3d(a, b):
    # TODO: This code is quite ugly
    x = dot_product(a, b) / (length_of_vector(a) * length_of_vector(b))
    return math.acos(x) * 180 / math.pi

def threepointangle(a, b, c):
    ba = create_vector_from_two_points(b, a)
    bc = create_vector_from_two_points(b, c)
    return angle3d(ba, bc)

def create_vector_from_two_points(a, b):
    return Vector3(b.x - a.x, b.y - a.y, b.z - a.z)

def dot_product(a, b):
    return a.x * b.x + a.y * b.y + a.z * b.z  # TODO: This currently "only" fits the first three dimensions

def length_of_vector(x):
    return math.sqrt(math.pow(x.x, 2) + math.pow(x.y, 2) + math.pow(x.z, 2))

if __name__ == '__main__':
    # initialize ros node
    rp.init_node('comparing_system_node', anonymous=False)

    # instantiate the Comparator class
    comparator = Comparator()

