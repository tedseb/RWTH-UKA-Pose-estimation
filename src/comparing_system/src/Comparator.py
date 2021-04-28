"""
This file contains the Comparator. 
"""


import threading
import math
import rospy as rp
import redis
import yaml
import time


from importlib import import_module
from threading import Thread
from functools import lru_cache

from src.config import *
from src.InterCom import *
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
    spot_queue_interface_class: type(SpotQueueInterface) = RedisSpotQueueInterface,
    feature_extractor_class: type(FeatureExtractor) = SpinFeatureExtractor):
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
                # Fetch all data that is needed for the comparison:
                spot_key = self.spot_queue_load_balancer.get_queue_key()

                _, _, spot_info_key, spot_state_key = generate_redis_key_names(spot_key)

                # Construct spot info dict, possibly from chache
                spot_info_dict = self.spot_info_interface.get_spot_info_dict(spot_info_key, ["exercise_data_hash", "start_time", "state", "repetitions"])
                
                # Use LRU Caching to update the spot info dict
                spot_info_dict.update(self.get_exercise_data(spot_info_key, spot_info_dict["exercise_data_hash"]))

                spot_state_dict = self.spot_info_interface.get_spot_state_dict(spot_state_key, spot_info_dict['exercise_data']['feature_of_interest_specification'])

                past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list = self.spot_queue_interface.dequeue(spot_key)

                # Compare joints with expert system data
                increase_reps, new_state, center_of_body = compare(spot_info_dict, spot_state_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list)

                spot_info_dict['state'] = new_state

                # Send info back back to outgoing message queue and back into the ROS system
                if increase_reps:
                    spot_info_dict['repetitions'] = int(spot_info_dict['repetitions']) + 1
                    user_state_message = {
                        'user_id': 0,
                        'current_exercise_name': spot_info_dict.get('exercise_data').get('name'),
                        'repetitions': spot_info_dict['repetitions'],
                        'seconds_since_last_exercise_start': (time.time() - int(spot_info_dict.get('start_time'))),
                        'milliseconds_since_last_repetition': 0,
                        'repetition_score': 100,
                        'exercise_score': 100
                    }
                    self.message_out_queue_interface.enqueue(REDIS_USER_STATE_SENDING_QUEUE_NAME, user_state_message)

                del spot_info_dict["exercise_data"]
                del spot_info_dict["exercise_data_hash"]
                del spot_info_dict['start_time']

                self.spot_info_interface.set_spot_info_dict(spot_info_key, spot_info_dict)

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


def compare(spot_info_dict: dict, spot_state_dict: dict, past_joints_with_timestamp_list: list, joints_with_timestamp: list, future_joints_with_timestamp_list: dict):
    exercise_data = spot_info_dict['exercise_data']
    
    joints = joints_with_timestamp['joints']
    timestamp = joints_with_timestamp['ros_timestamp']

    
    print(joints_with_timestamp)
    print(spot_state_dict)

    pose = {}

    for index in joints_used:
        point = Point()
        # This code currently swaps Y and Z axis, which is how Tamer did this. # TODO: Find defenitive solution to this
        point.x = joints[index]['point']['x']
        point.y = joints[index]['point']['z']
        point.z = joints[index]['point']['y']
        pose[joint_labels[index]] = point

    repetition_counted, new_state = count(exercise_data, spot_state_dict, pose)

    # We define the center of the body as the pelvis
    center_of_body = pose[center_of_body_label]

    return repetition_counted, new_state, center_of_body

def count(exercise_data, state_dict, pose):
    """
    Check if the state has changed. If so, possibly increment repetitions and return them. Otherwise return None
    """
    boundaries = exercise_data['boundaries']

    new_state_dict = {}

    # print(boundaries)
    # print(state_dict)

    # CALCULATE NEW STATES HERE AND COMPARE THEM TO OLD STATES

    for joint_names, angle_info_dict in boundaries["angles"]["angles_high"].items():
        if calculateAngle(angle_info_dict["inner_joint"], angle_info_dict["outer_joints"], pose) > angle_info_dict["angle"]:
            
            return "high" == exercise_data['beginning_state_dict'], "high"
    else:
        return False, "low"

    for joint_names, angle_info_dict in boundaries["angles"]["angles_low"].items():
        if calculateAngle(angle_info_dict["inner_joint"], angle_info_dict["outer_joints"], pose) < angle_info_dict["angle"]:
            return "low" == exercise_data['beginning_state_dict'], "low"
    else:
        return False, "high"
    

def checkforstate(this_angle, angle, state):
    return (this_angle >= angle - alpha and this_angle <= angle + alpha)
