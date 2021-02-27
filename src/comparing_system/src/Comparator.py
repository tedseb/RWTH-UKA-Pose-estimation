"""
This file contains the Comparator. 
"""


import threading

import math
import numpy as np
import rospy as rp
import redis

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
corrections = dict()
reps = 0

class NoJointsAvailable(Exception):
    pass

class NoSpotInfoAvailable(Exception):
    pass

class Comparator(Thread):
    def __init__(self, message_queue_load_order, user_state_out_queue, user_correction_out_queue, exercises, redis_connection_pool):
        super(Comparator, self).__init__()

        self.message_queue_load_order = message_queue_load_order
        self.user_state_out_queue = user_state_out_queue
        self.user_correction_out_queue = user_correction_out_queue
        
        # This datafield is filled with exercises retrieved from tamer by the ExerciseDataUpdater
        self.exercises = exercises
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)

        self.running = True

        self.start()

    def run(self):
        """
        This is our main threading loop. We devide it into three parts for a  better overview:
            * Getting data from the queues with self.get_data()
            * Comparing the "is" and "should be" data for the joints with self.compare()
            * Putting data back into sending queues with self.send_info()
        """
        while(self.running):
            try:
                spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list = self.get_data()
            except NoJointsAvailable:
                continue
            except NoSpotInfoAvailable:
                continue
            except Exception as e:
                traceback.print_exc() 
                rp.logerr("Error getting data in the comparator: " + str(e))
            try:
                info = self.compare(spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list)
            except Exception as e:
                traceback.print_exc() 
                rp.logerr("Error comparing data in the comparator: " + str(e))
            try:
                self.send_info(info, spot_info_dict)
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
            raise NoJointsAvailable

        redis_spot_queue_key = redis_spot_key + ':queue'
        redis_spot_info_key = redis_spot_key + ':info'
        redis_spot_past_queue_key = redis_spot_queue_key + "_past"
        
        try:
            joinsts_with_timestamp_yaml = self.redis_connection.rpop(redis_spot_queue_key) # self.redis_connection.rpoplpush(redis_spot_queue_key, redis_spot_past_queue_key)
            if not joinsts_with_timestamp_yaml:
                raise KeyError
        except KeyError:
            # Supposingly, no message queue is holding any value (at the start of the system)
            raise NoJointsAvailable
    
        joints_with_timestamp = yaml.load(joinsts_with_timestamp_yaml)

        redis_spot_info_yaml = self.redis_connection.get(redis_spot_info_key)
        if redis_spot_info_yaml == None:
            raise NoSpotInfoAvailable

        spot_info_dict = yaml.load(redis_spot_info_yaml) # TODO: Switch from using yaml to Rejson
        exercise = spot_info_dict['exercise']
        start_time = spot_info_dict['start_time']

        self.redis_connection.ltrim(redis_spot_past_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)

        future_joints_with_timestamp_list = None # self.redis_connection.lrange(redis_spot_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)
        past_joints_with_timestamp_list = None # self.redis_connection.lrange(redis_spot_past_queue_key, 0, REDIS_MAXIMUM_PAST_QUEUE_SIZE)

        return spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list

    def send_info(self, info, spot_info_dict):

        updated_repetitions, correction = info

        if correction != None:
            user_info_message = {
                'user_id': 0,
                'repetition': repetitions,
                'positive_correction': False,
                'display_text': correction
            }
            self.redis_connection.lpush(REDIS_USER_INFO_SENDING_QUEUE_NAME, yaml.dump(user_info_message))
        
        if updated_repetitions != None:
            user_state_message = {
                'user_id': 0,
                'current_exercise_name': spot_info_dict.get('exercise').get('name'),
                'repetitions': repetitions,
                'seconds_since_last_exercise_start': (rp.Time.now() - spot_info_dict.get('start_time')).to_sec(),
                'milliseconds_since_last_repetition': 0,
                'repetition_score': 100,
                'exercise_score': 100,
                'user_position': {'x':0, 'y':0, 'z': 0}
            }
            self.redis_connection.lpush(REDIS_USER_STATE_SENDING_QUEUE_NAME, yaml.dump(user_state_message))
        


    def compare(self, spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list):
        global lastPose
        current_exercise = spot_info_dict['exercise']
        
        joints = joints_with_timestamp['joints']
        timestamp = joints_with_timestamp['timestamp']

        if current_exercise['name'] not in corrections.keys():
            corrections[current_exercise['name']] = dict()

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

        updated_repetitions = self.count(current_exercise)
        correction = self.correct(angles, current_exercise, state)
        return updated_repetitions, correction

        
    def correct(self, angles, exercise, state):
        messages = self.checkForCorrection(angles, exercise['stages'], state)
        if (messages != corrections[exercise['name']].get(state)):
            corrections[exercise['name']][state] = messages
            if (len(messages) > 0):
                # self.error_publisher.publish(messages)
                return messages
    
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
            elif rule == "min":
                if (angles[key] <= rules[key][1] - alpha):
                    corrections += rules[key][2] + ". "
                    # console.log(JSON.stringify(angle_points[k]))
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
        return corrections

    def count(self, current_exercise):
        """
        Check if the state has changed. If so, possibly increment repetitions and return them. Otherwise return None
        """
        
        # TODO: Get these global Variables into a class
        global state
        global reps

        if state < 2 and checkforstate(angles, current_exercise, state + 1):
            state += 1
        
        if (state == current_exercise['stages']):
            if (checkforstate(angles, current_exercise, 0)):
                state = 0
                reps += 1
                return reps

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


