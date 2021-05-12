"""
This file contains the Comparator.
A Comparator thread dequeues data from spots, extracts features and compares them to exercises performed
by experts, i.e. data from the expert system.
Whatever information is gained is sent via an outgoing message queue.
"""

import math
import threading
import time
import hashlib
from functools import lru_cache
from importlib import import_module
from threading import Thread
from traceback import print_exc
from typing import NoReturn

import redis
import rospy as rp
import numpy as np
import yaml
from src.config import *
from src.FeatureExtraction import *
from src.InterCom import *
from src.Util import *


class NoJointsAvailable(Exception):
    pass

class NoSpoMetaDataAvailable(Exception):
    pass

class NoExerciseDataAvailable(Exception):
    pass

class MalformedFeatures(Exception):
    pass

# TODO: Divide into stage one and stage two for counting and comparing
class Comparator(Thread):
    """
    The Comparator can be scaled horizontally. It pops data from inbound spot queues and calculates and metrics which are put into outbound message queues.
    """
    def __init__(self, 
    spot_metadata_interface_class: type(SpotMetaDataInterface) = RedisSpotMetaDataInterface, 
    spot_queue_load_balancer_class: type(QueueLoadBalancerInterface) = RedisQueueLoadBalancerInterface, 
    message_out_queue_interface_class: type(MessageQueueInterface) = RedisMessageQueueInterface, 
    spot_queue_interface_class: type(SpotQueueInterface) = RedisSpotQueueInterface,
    feature_extractor_class: type(FeatureExtractor) = SpinFeatureExtractor,
    feature_data_queues_interface_class: type(PastFeatureDataQueuesInterface) = RedisFeatureDataQueuesInterface):
        super(Comparator, self).__init__()

        self.spot_queue_load_balancer = spot_queue_load_balancer_class()
        self.message_out_queue_interface = message_out_queue_interface_class()
        self.spot_queue_interface = spot_queue_interface_class()
        self.spot_metadata_interface = spot_metadata_interface_class()
        self.feature_extractor = feature_extractor_class()
        # Use the same interface class for these two, because they have the same needs to the interface 
        self.past_features_queue_interface = feature_data_queues_interface_class()
        self.past_resampled_features_queue_interface = feature_data_queues_interface_class()
        
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)

        # This can be set to false by an external entity to stop the loop from running
        self.running = True

        self.start()

    @lru_cache(maxsize=EXERCISE_DATA_LRU_CACHE_SIZE)
    def get_exercise_data(self, spot_key, exercise_data_hash):
        return self.spot_metadata_interface.get_spot_info_dict(spot_key, ["exercise_data"])

    def run(self) -> NoReturn:
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

                _, _, spot_info_key, spot_state_key, spot_feature_progression_key, spot_resampled_features_key = generate_redis_key_names(spot_key)

                # Construct spot info dict, possibly from chache
                spot_info_dict = self.spot_metadata_interface.get_spot_info_dict(spot_info_key, ["exercise_data_hash", "start_time", "repetitions"])
                
                # Use LRU Caching to update the spot info dict
                spot_info_dict.update(self.get_exercise_data(spot_info_key, spot_info_dict["exercise_data_hash"]))

                past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list = self.spot_queue_interface.dequeue(spot_key)

                last_feature_progressions = self.past_features_queue_interface.get_features(spot_feature_progression_key, latest_only=True)
                last_resampled_features = self.past_resampled_features_queue_interface.get_features(spot_resampled_features_key, latest_only=True)

                # Compare joints with expert system data
                increase_reps, new_features_progression, new_resampled_features = self.compare_high_level_features(spot_info_dict, last_feature_progressions, last_resampled_features, joints_with_timestamp)

                self.past_features_queue_interface.enqueue(spot_feature_progression_key, new_features_progression)
                self.past_resampled_features_queue_interface.enqueue(spot_resampled_features_key, new_resampled_features)

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

                    self.spot_metadata_interface.set_spot_info_dict(spot_info_key, {"repetitions": spot_info_dict['repetitions']})


                # Calculate a new reference pose mapping
                if new_resampled_features:
                    all_resampled_features = self.past_resampled_features_queue_interface.get_features(spot_resampled_features_key, latest_only=False)
                    reference_pose = self.calculate_reference_pose_mapping(all_resampled_features, spot_info_dict['exercise_data'])

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

    def compare_high_level_features(self, 
    spot_info_dict: dict, 
    last_feature_progressions: dict,
    last_resampled_features: dict,
    joints_with_timestamp: list) -> Tuple[bool, dict, Any]:
        """Compare high level features, such as angles, by extracting them from the joints array and turn them into a progression."""
        exercise_data = spot_info_dict['exercise_data']
        used_joint_ndarray = joints_with_timestamp['used_joint_ndarray']
        timestamp = joints_with_timestamp['ros_timestamp']
        beginning_states = spot_info_dict['exercise_data']['beginning_state_dict']

        features_states = self.feature_extractor.extract_states(used_joint_ndarray, exercise_data['reference_feature_data'], exercise_data['feature_of_interest_specification'])

        def copmute_new_feature_progression(beginning_state, features_state, last_feature_progression):
            new_feature_progression = last_feature_progression

            if beginning_state == FEATURE_HIGH:
                if features_state == FEATURE_HIGH:
                    feature_is_in_beginning_state = True
                elif features_state == FEATURE_LOW:
                    new_feature_progression  = PROGRESSION_PARTIAL
                    feature_is_in_beginning_state = False
                else:
                    feature_is_in_beginning_state = False
            elif beginning_state == FEATURE_LOW:
                if features_state == FEATURE_LOW:
                    feature_is_in_beginning_state = True
                elif features_state == FEATURE_HIGH:
                    new_feature_progression  = PROGRESSION_PARTIAL
                    feature_is_in_beginning_state = False
                else:
                    feature_is_in_beginning_state = False
            else:
                raise MalformedFeatures("Beginning state is " + str(beginning_state))
            
            if feature_is_in_beginning_state:
                if last_feature_progression in (PROGRESSION_DONE, PROGRESSION_PARTIAL):
                    new_feature_progression  = PROGRESSION_DONE
                else:
                    new_feature_progression = PROGRESSION_START

            return new_feature_progression

        increase_reps = True
        new_feature_progressions = {}
        new_resampled_features = {}

        for feature_type, features in features_states.items():
            new_feature_progressions[feature_type] = {}
            new_resampled_features[feature_type] = {}
            for k, v in features.items():
                # With the current feature value and the last resampled feature value, we can compute new resampled feature values
                feature_value = features_states[feature_type][k]['feature_value']
                resolution = exercise_data['reference_feature_data'][feature_type][k]['range_of_motion'] * FEATURE_TRAJECTORY_RESOLUTION_FACTOR
                scale = exercise_data['reference_feature_data'][feature_type][k]['resampled_reference_trajectory_scale_array']
                try:
                    last_resampled_feature_value = float(last_resampled_features[feature_type][k])
                    new_resampled_feature_values = compute_resampled_feature_values(feature_value, last_resampled_feature_value, resolution)
                    if new_resampled_feature_values:
                        new_resampled_features[feature_type][k] = new_resampled_feature_values
                except KeyError:
                    # If we have no resampled feature values yet, set them to the nearest resampled one
                    new_resampled_features[feature_type][k] = scale[np.argmin(abs(scale - feature_value))]
                
                # With the beginning state of a feature and the current feature state, we can computer the new feature progression value
                try:
                    last_feature_progression = last_feature_progressions[feature_type][k]
                except KeyError:
                    # If we have no last feature progression value, set this feature progression to the starting value
                    last_feature_progression = PROGRESSION_START
                beginning_state = beginning_states[feature_type][k]['feature_state']
                features_state = features_states[feature_type][k]['feature_state']
                new_feature_progression = copmute_new_feature_progression(beginning_state, features_state, last_feature_progression)
                
                # If one of the features is not done, the repetition is not done
                if new_feature_progression != PROGRESSION_DONE:
                    increase_reps = False
                # We only need to update the feature progression state if the progress has changed
                if (new_feature_progression != last_feature_progression):
                    new_feature_progressions[feature_type][k] = new_feature_progression
            # If a data type has no updates, remove it again
            if new_feature_progressions[feature_type] == {}:
                del new_feature_progressions[feature_type]
            if new_resampled_features[feature_type] == {}:
                del new_resampled_features[feature_type]
        
        # TODO: Do something safe here, this might not reset all features
        if increase_reps:
            def reset_child_featuers(d):
                for k, v in d.items():
                    if isinstance(v, collections.MutableMapping):
                        d[k] = reset_child_featuers(v)
                    else:
                        d[k] = PROGRESSION_START
                    return d
            new_feature_progressions = reset_child_featuers(new_feature_progressions)

        return increase_reps, new_feature_progressions, new_resampled_features

    def calculate_reference_pose_mapping(self, feature_trajectories, exercise_data) -> np.ndarray:
        reference_poses = exercise_data['recording']

        # Collect all feature trajectories from the exercise data dict and turn them into a numpy array
        all_reference_feature_data_dicts = dict()
        for feature_type, features in exercise_data['reference_feature_data'].items():
            for k, v in features.items():
                resampled_values_reference_trajectory_indices = v['resampled_values_reference_trajectory_indices']
                reference_trajectory_hankel_matrices = v['reference_trajectory_hankel_matrices']
                feature_trajectory = np.asarray(feature_trajectories[feature_type][k], np.float16)

                # TODO: Do this for all hankel matrices
                for reference_trajectory_hankel_matrix in reference_trajectory_hankel_matrices:
                    errors = custom_metric(reference_trajectory_hankel_matrix, feature_trajectory, 0.5)
                    prediction = np.argmin(errors)

                    # if last_prediction == None:
                    #     last_prediction = prediction

                    # if prediction > self.last_prediction:
                    #     prediction = ((1 - elasticity) * self.last_prediction + elasticity * prediction)

                    # last_prediction = prediction



def custom_metric(hankel_matrix, feature_trajectory, beta):
    """
    Return the l2 norm of signals - signal, but influence fades out linearly form the 
    newest to the oldest measurement according to beta.
    """
    comparing_length = min((len(feature_trajectory), len(hankel_matrix)))
    hankel_matrix_shortened = hankel_matrix[:, :comparing_length]
    feature_trajectory_shortened = feature_trajectory[:comparing_length]
    distances = hankel_matrix_shortened - feature_trajectory_shortened
    fading_factor = np.linspace(beta, 1, comparing_length) # Let older signals have less influence on the error
    errors = np.linalg.norm((distances) * fading_factor, axis=1)
    return errors

