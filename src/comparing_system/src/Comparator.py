"""
This file contains the Comparator. 
"""


import threading
import math
import rospy as rp
import redis
import yaml

from importlib import import_module
from threading import Thread
from functools import lru_cache

from src.joint_adapters.spin import *
from src.config import *
from src.InterCom import *
from src.LegacyCompaing import compare_legacy
from src.FeatureExtraction import *

from traceback import print_exc

import time


class NoJointsAvailable(Exception):
    pass

class NoSpotInfoAvailable(Exception):
    pass

class NoExerciseDataAvailable(Exception):
    pass

class Comparator(Thread):
    """
    The Comparator can be scaled horizontally. It pops data from inbound spot queues and calculates and metrics which are put into outbound message queues.
    """
    def __init__(self, 
    spot_info_interface_class: type(SpotInfoInterface) = RedisSpotInfoInterface, 
    spot_queue_load_balancer_class: type(QueueLoadBalancerInterface) = RedisQueueLoadBalancerInterface, 
    message_out_queue_interface_class: type(MessageQueueInterface) = RedisMessageQueueInterface, 
    spot_queue_interface_class: type(SpotQueueInterface) = RedisSpotQueueInterface):
        super(Comparator, self).__init__()

        self.spot_queue_load_balancer = spot_queue_load_balancer_class()
        self.message_out_queue_interface = message_out_queue_interface_class()
        self.spot_queue_interface = spot_queue_interface_class()
        self.spot_info_interface = spot_info_interface_class()
        
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)

        # This can be set to false by an external entity to stop the loop from running
        self.running = True

        self.start()

    @lru_cache(maxsize=EXERCISE_DATA_LRU_CACHE_SIZE)
    def get_exercise_data(self, spot_key, exercise_data_hash):
        return self.spot_info_interface.get_spot_info_dict(spot_key, ["exercise_data"])

    def run(self):
        """
        This is our main threading loop. We devide it into three parts for a  better overview:
            * Getting data from the queues with self.get_data()
            * Comparing the "is" and "should be" data for the joints with self.compare()
            * Putting data back into sending queues with self.send_info()
        """
        while(self.running):
            try:
                start = time.time()
                
                # Fetch all data that is needed for the comparison:
                spot_key = self.spot_queue_load_balancer.get_queue_key()

                spot_queue_key, spot_past_queue_key, spot_info_key = generate_redis_key_names(spot_key)

                _1 = time.time()
                

                # Construct spot info dict, possibly from chache
                spot_info_dict = self.spot_info_interface.get_spot_info_dict(spot_info_key, ["exercise_data_hash", "start_time", "state", "repetitions"])
                exercise_data = self.get_exercise_data(spot_info_key, spot_info_dict["exercise_data_hash"])
                spot_info_dict.update(exercise_data) 
                
                _2 = time.time()
                print("start:" + str(start))
                print("Getting the spot key:" + str(_1 - start))
                print("Getting the spot info dict:" + str(_2 - _1))

                past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list = self.spot_queue_interface.dequeue(spot_key)
                
                _3 = time.time()
                print("Dequeueing from the spot queue interface:" + str(_3 - _2))
                
                # Compare joints with expert system data
                increase_reps, new_state, center_of_body = self.compare(spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list)

                _4 = time.time()
                print("Comparing:" + str(_4 - _3))

                spot_info_dict['state'] = new_state

                # Send info back back to outgoing message queue and back into the ROS system
                if increase_reps:
                    spot_info_dict['repetitions'] += 1
                    user_state_message = {
                        'user_id': 0,
                        'current_exercise_name': spot_info_dict.get('exercise').get('name'),
                        'repetitions': spot_info_dict['repetitions'],
                        'seconds_since_last_exercise_start': (rp.Time.now().secs - spot_info_dict.get('start_time')),
                        'milliseconds_since_last_repetition': 0,
                        'repetition_score': 100,
                        'exercise_score': 100
                    }
                    self.message_out_queue_interface.enqueue(REDIS_USER_STATE_SENDING_QUEUE_NAME, user_state_message)
                
                self.spot_info_interface.set_spot_info_dict(spot_key, spot_info_dict)

                # Corrections are not part of the alpha release, we therefore leave them out and never send user correction messages
                correction = None

                if correction != None and SEND_CORRETIONS:
                    user_correction_message = {
                        'user_id': 0,
                        'repetition': spot_info_dict['repetitions'],
                        'positive_correction': False,
                        'display_text': correction
                    }
                    self.message_out_queue_interface.enqueue(REDIS_USER_INFO_SENDING_QUEUE_NAME, user_correction_message)

            except QueueEmpty:
                continue
            except Exception as e:
                if HIGH_VERBOSITY:
                    print_exc() 
                    rp.logerr("Error sending data in the comparator: " + str(e))    
        
    def compare(self, spot_info_dict: dict, past_joints_with_timestamp_list: list, joints_with_timestamp: list, future_joints_with_timestamp_list: dict):
        if LEGACY_COMPARING:
            return compare_legacy(spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list)
        else:
            return compare(spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list)


def compare(spot_info_dict: dict, past_joints_with_timestamp_list: list, joints_with_timestamp: list, future_joints_with_timestamp_list: dict):
    exercise_data = spot_info_dict['exercise_data']
    state = spot_info_dict['state']
    
    joints = joints_with_timestamp['joints']
    timestamp = joints_with_timestamp['timestamp']

    pose = {}

    for index in joints_used:
        point = Point()
        # This code currently swaps Y and Z axis, which is how Tamer did this. # TODO: Find defenitive solution to this
        point.x = joints[index]['point']['x']
        point.y = joints[index]['point']['z']
        point.z = joints[index]['point']['y']
        pose[joint_labels[index]] = point

    repetition_counted, new_state = count(exercise_data, state, pose)

    # We define the center of the body as the pelvis
    center_of_body = pose[center_of_body_label]

    return repetition_counted, new_state, center_of_body

def count(exercise_data, state, pose):
    """
    Check if the state has changed. If so, possibly increment repetitions and return them. Otherwise return None
    """
    boundaries = exercise_data['boundaries']

    if state == "low":
        for joint_names, angle_info_dict in boundaries["angles"]["angles_high"].items():
            if calculateAngle(angle_info_dict["inner_joint"], angle_info_dict["outer_joints"], pose) > angle_info_dict["angle"]:
                return "high" == exercise_data['beginning_state'], "high"
        else:
            return False, "low"
    elif state == "high":
        for joint_names, angle_info_dict in boundaries["angles"]["angles_low"].items():
            if calculateAngle(angle_info_dict["inner_joint"], angle_info_dict["outer_joints"], pose) < angle_info_dict["angle"]:
                return "low" == exercise_data['beginning_state'], "low"
        else:
            return False, "high"
    else:
        raise Exception

def checkforstate(this_angle, angle, state):
    return (this_angle >= angle - alpha and this_angle <= angle + alpha)

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