"""
This file contains the Comparator. The comparator is a thread that handles Redis FIFO Queues that contain joints that are produced by whatever AI our System uses.
Data that is expected by the comparator is listed below.

The redis keys and values are as follows:
    * Spot with ID N is:                <spot #N>               :(prefix for other data)
    * Queue for spot with ID N is       <spot #N:queue>         :Redis FIFO Queue
    * Past queue for spot with ID N is  <spot #N:queue_past>    :Redis FIFO Queue
    * Info for spot with ID N is        <spot #N:info>          :stringified YAML

The 'queue' for a spot is expected to contain a list of of dictionaries '{'bodyParts': <Bodyparts>, 'time_stamp': <header.stamp>}'.
These dictionaries represent data not yet processed by the comparator, or 'the future'.

The 'past queue' for a spot is expected to contains the dictionaries already processed, or 'the past'.

The info for a spot contains additional information like timing data in a dictionary.
"""


import threading

import math
import numpy as np
import rospy as rp

from importlib import import_module
from collections import deque
from threading import Thread
import yaml
from src.joint_adapters.spin import *
from src.config import *

import traceback 


#Old imports 

from backend.msg import Persons
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

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


class Comparator(Thread):
    def __init__(self, message_queue_load_order, user_state_out_queue, user_correction_out_queue, exercises, redis_connection):
        super(Comparator, self).__init__()

        self.message_queue_load_order = message_queue_load_order
        self.user_state_out_queue = user_state_out_queue
        self.user_correction_out_queue = user_correction_out_queue
        
        # This datafield is filled with exercises retrieved from tamer by the ExerciseDataUpdater
        self.exercises = exercises
        self.redis_connection = redis_connection

        self.running = True

        self.start()


    def run(self):
        while(self.running):
            try:
                spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list = self.get_data()
            except Exception as e:
                traceback.print_exc() 
                rp.logerr("Error getting data in the comparator: " + str(e))
            try:
                info = self.compare(spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list)
            except Exception as e:
                traceback.print_exc() 
                rp.logerr("Error comparing data in the comparator: " + str(e))
            try:
                self.send_info(info)
            except Exception as e:
                traceback.print_exc() 
                rp.logerr("Error sending data in the comparator: " + str(e))
                
            

    def get_data(self):
        # TODO: Investigate if these redis instructions can be optimized
        # Fetch all data that is needed for the comparison:
        try:
            redis_spot_key, redis_spot_queue_length = self.message_queue_load_order.popitem()
        except KeyError:
            # Supposingly, no message queue is holding any value (at the start of the system)
            return

        redis_spot_queue_key = redis_spot_key + ':queue'
        redis_spot_info_key = redis_spot_key + ':info'
        redis_spot_past_queue_key = redis_spot_queue_key + "_past"
        
        joints_with_timestamp = yaml.load(self.redis_connection.rpoplpush(redis_spot_queue_key, redis_spot_past_queue_key))
        
        spot_info_dict = yaml.load(self.redis_connection.get(redis_spot_info_key)) # TODO: Switch from using yaml to Rejson
        exercise = spot_info_dict['exercise']
        start_time = spot_info_dict['start_time']

        self.redis_connection.ltrim(redis_spot_past_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)

        future_joints_with_timestamp_list = self.redis_connection.lrange(redis_spot_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)
        past_joints_with_timestamp_list = self.redis_connection.lrange(redis_spot_past_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)

        return spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list

    def send_info(self, info):

        dummy_user_info = {
            'user_id': 1,
            'repetition': 0,
            'positive_correction': False,
            'display_text': "Hi Orhan"
        }

        dummy_user_state = {
            'user_id': 1,
            'current_exercise_name': "testexercisename",
            'repetitions': 0,
            'seconds_since_last_exercise_start': 1,
            'milliseconds_since_last_repetition': 1,
            'repetition_score': 100,
            'exercise_score': 100,
            'user_position': {'x':0, 'y':0, 'z': 0}
        }
        
        self.redis_connection.lpush(REDIS_USER_STATE_SENDING_QUEUE_NAME, yaml.dump(dummy_user_state))
        self.redis_connection.lpush(REDIS_USER_INFO_SENDING_QUEUE_NAME, yaml.dump(dummy_user_info))


    def compare(self, spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list):
        global lastPose
        current_exercise = spot_info_dict['exercise']
        
        joints = joints_with_timestamp['joints']
        timestamp = joints_with_timestamp['timestamp']

        if current_exercise['name'] not in states.keys():
            states[current_exercise['name']] = []
        
        if current_exercise['name'] not in corrections.keys():
            corrections[current_exercise['name']] = {1: "", 2: ""}

        pose = {}

        for index in ownpose_used:
            point = Point()
            # This code currently swaps Y and Z axis, which is how Tamer did this. # TODO: Find defenitive solution to this
            point.x = joints[index].point.x
            point.y = joints[index].point.z
            point.z = joints[index].point.y
            pose[joint_labels[index]] = point

        # This corresponds to Tamers "save" method
        last_30_poses.append(pose)
        lastPose = pose
        if (len(last_30_poses) >= 30):
            last_30_poses.popleft()
            # console.log(checkForStretch(last30)) # TODO: This is nodejs code from Tamer, find out what it does

        for angle, points in angle_joints_mapping.items():
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

        self.count(current_exercise)
        self.correct(angles, current_exercise, state)
        
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
                    self.coordinates_publisher.publish(str(angle_joints_mapping[key]))
            elif rule == "min":
                if (angles[key] <= rules[key][1] - alpha):
                    corrections += rules[key][2] + ". "
                    # console.log(JSON.stringify(angle_points[k]))
                    self.coordinates_publisher.publish(str(angle_joints_mapping[key]))
            elif rule == "behind":
                # a: 0, b: 1, p: 1, x: 2
                a = lastPose[angle_joints_mapping[key][0]]
                b = lastPose[angle_joints_mapping[key][1]]
                p = lastPose[angle_joints_mapping[key][1]]
                x = lastPose[angle_joints_mapping[key][2]]
                val_x = plane3d(a, b, p, x)
                val_a = plane3d(a, b, p, a)
                if (math.copysign(1, val_x) == math.copysign(1, val_a)):
                    corrections += rules[key][2] + ". "
                    self.coordinates_publisher.publish(str(angle_joints_mapping[key]))
        return corrections

    def count(self, current_exercise):
        # TODO: Get these global Variables into a class
        global state
        global reps
        global states

        if state < 3 and checkforstate(angles, current_exercise, state + 1):
            # states.squats.push(states + 1)
            state += 1
        
        if (state == 3):
            if (checkforstate(angles, current_exercise, 1)):
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
    for key in angle_joints_mapping.keys():
        differences[key] = []
    for element in arr:
        for angle, points in angle_joints_mapping:
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


