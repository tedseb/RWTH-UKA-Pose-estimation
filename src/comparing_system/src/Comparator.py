"""
This file contains the Comparator.
A Comparator thread dequeues data from spots, extracts features and compares them to exercises performed
by experts, i.e. data from the expert system.
Whatever information is gained is sent via an outgoing message queue.
"""

import math
import threading
import time
from functools import lru_cache
from importlib import import_module
from threading import Thread
from traceback import print_exc

import redis
import rospy as rp
import yaml
from src.config import *
from src.FeatureExtraction import *
from src.InterCom import *


class NoJointsAvailable(Exception):
    pass

class NoSpoMetaDataAvailable(Exception):
    pass

class NoExerciseDataAvailable(Exception):
    pass

class Comparator(Thread):
    """
    The Comparator can be scaled horizontally. It pops data from inbound spot queues and calculates and metrics which are put into outbound message queues.
    """
    def __init__(self, 
    spot_metadata_interface_class: type(SpotMetaDataInterface) = RedisSpotMetaDataInterface, 
    spot_queue_load_balancer_class: type(QueueLoadBalancerInterface) = RedisQueueLoadBalancerInterface, 
    message_out_queue_interface_class: type(MessageQueueInterface) = RedisMessageQueueInterface, 
    spot_queue_interface_class: type(SpotQueueInterface) = RedisSpotQueueInterface,
    feature_extractor_class: type(FeatureExtractor) = SpinFeatureExtractor):
        super(Comparator, self).__init__()

        self.spot_queue_load_balancer = spot_queue_load_balancer_class()
        self.message_out_queue_interface = message_out_queue_interface_class()
        self.spot_queue_interface = spot_queue_interface_class()
        self.spot_metadata_interface = spot_metadata_interface_class()
        self.feature_extractor = feature_extractor_class()
        
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)

        # This can be set to false by an external entity to stop the loop from running
        self.running = True

        self.start()

    @lru_cache(maxsize=EXERCISE_DATA_LRU_CACHE_SIZE)
    def get_exercise_data(self, spot_key, exercise_data_hash):
        return self.spot_metadata_interface.get_spot_info_dict(spot_key, ["exercise_data"])

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
                spot_info_dict = self.spot_metadata_interface.get_spot_info_dict(spot_info_key, ["exercise_data_hash", "start_time", "repetitions"])
                
                # Use LRU Caching to update the spot info dict
                spot_info_dict.update(self.get_exercise_data(spot_info_key, spot_info_dict["exercise_data_hash"]))

                spot_state_dict = self.spot_metadata_interface.get_spot_state_dict(spot_state_key, spot_info_dict['exercise_data']['feature_of_interest_specification'])

                past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list = self.spot_queue_interface.dequeue(spot_key)

                # Compare joints with expert system data
                increase_reps, new_state, center_of_body = self.compare(spot_info_dict, spot_state_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list)

                self.spot_metadata_interface.set_spot_state_dict(spot_state_key, new_state)

                # Send info back back to outgoing message queue and back into the ROS system
                if increase_reps:
                    spot_info_dict['repetitions'] = int(spot_info_dict['repetitions']) + 1
                    user_state_message = {
                        'user_id': 0,
                        'current_exercise_name': spot_info_dict.get('exercise_data').get('name'),
                        'repetitions': spot_info_dict['repetitions'],
                        'seconds_since_last_exercise_start': (time.time_ns() - int(spot_info_dict.get('start_time'))) / 1e+9,
                        'milliseconds_since_last_repetition': 0,
                        'repetition_score': 100,
                        'exercise_score': 100
                    }
                    self.message_out_queue_interface.enqueue(REDIS_USER_STATE_SENDING_QUEUE_NAME, user_state_message)

                del spot_info_dict["exercise_data"]
                del spot_info_dict["exercise_data_hash"]
                del spot_info_dict['start_time']

                self.spot_metadata_interface.set_spot_info_dict(spot_info_key, spot_info_dict)

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
            except SpotMetaDataException as e:
                if HIGH_VERBOSITY:
                    rp.logerr(e)    
            except Exception as e:
                if HIGH_VERBOSITY:
                    print_exc() 
                    rp.logerr("Encountered an Error while Comparing: " + str(e))    


    def compare(self, spot_info_dict: dict, spot_state_dict: dict, past_joints_with_timestamp_list: list, joints_with_timestamp: list, future_joints_with_timestamp_list: dict) -> Tuple[bool, dict, Any]:
        exercise_data = spot_info_dict['exercise_data']
        
        used_joint_ndarray = joints_with_timestamp['used_joint_ndarray']
        timestamp = joints_with_timestamp['ros_timestamp']

        new_spot_state_dict = self.feature_extractor.extract_states(used_joint_ndarray, exercise_data['boundaries'], exercise_data['feature_of_interest_specification'])

        increase_reps = True

        for feature_type, features in new_spot_state_dict.items():
            for k, v in features.items():
                if spot_state_dict[feature_type][k] == "done":
                    new_spot_state_dict[feature_type][k] = "done"
                    continue
                if v == spot_info_dict['exercise_data']['beginning_state_dict'][feature_type][k] and v != spot_state_dict[feature_type][k]:
                    new_spot_state_dict[feature_type][k] = "done"
                else:
                    increase_reps = False
        
        # TODO: We define the center of the body as the pelvis
        center_of_body = None

        if increase_reps:
            new_spot_state_dict = spot_info_dict['exercise_data']['beginning_state_dict']

        return increase_reps, new_spot_state_dict, center_of_body

