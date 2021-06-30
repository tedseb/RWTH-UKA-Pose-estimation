#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Spot Comparator Thread.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
The Node spawns mutiple Comparator threads.
A Comparator thread dequeues data from spot queues, extracts features and compares them to exercises performed
by experts, i.e. data from the expert system.
Whatever information is gained is sent via outgoing message queues.
"""

import queue
import time
from functools import lru_cache
from threading import Thread
from traceback import print_exc
from typing import NoReturn
from queue import Queue

import collections
import redis
import rospy as rp
import numpy as np
from std_msgs.msg import String

try:
    from comparing_system.src.config import *
    from comparing_system.src.FeatureExtraction import *
    from comparing_system.src.InterCom import *
    from comparing_system.src.Util import *
except ImportError:
    from src.config import *
    from src.FeatureExtraction import *
    from src.InterCom import *
    from src.Util import *

from backend.msg import Person


class NoJointsAvailable(Exception):
    pass

class NoSpoMetaDataAvailable(Exception):
    pass

class NoExerciseDataAvailable(Exception):
    pass


def custom_metric(hankel_matrix, feature_trajectory, max_weight_, min_weight):
    """Compute a custom metric that represents how probable it is for the user to be at a certain point of the reference trajectory.

    This function computes the l2 norm of signals - signal, but influence to the error fades out linearly form the 
    newest to the oldest measurement according to beta.

    Args:
        hankel_matrix: The hankel matrix of the reference trajectory.
        feature_trajectory: The trajectory of a feature of the user that we want to compare against
        max_weight_: Dictates how strong the newest values are weighted
        min_weight: Dictates how weak the oldest values are weighted
    
    Returns:
        An error for every step in the feature_trajectory
    """
    comparing_length = min((len(feature_trajectory), len(hankel_matrix)))
    hankel_matrix_shortened = hankel_matrix[:, :comparing_length]
    feature_trajectory_shortened = feature_trajectory[-comparing_length:]
    np.flip(feature_trajectory_shortened)
    distances = hankel_matrix_shortened - feature_trajectory_shortened
    fading_factor = np.linspace(min_weight, max_weight_, comparing_length) # Let older signals have less influence on the error
    errors = np.linalg.norm((distances) * fading_factor, axis=1)
    return errors


def compare_high_level_features(spot_info_dict: dict, 
    last_feature_progressions: dict,
    last_resampled_features: dict,
    features_states: dict,
    bad_repetition: bool) -> Tuple[bool, bool, dict, Any]:
    """Compare high level features, such as angles, by extracting them from the joints array.
    
    This method turn high level features into a progression dictionary and resamples them.
    By calculating the progression dictionary it also detects repetitions.

    Args:
        spot_info_dict: Contains the exercise data that we want to compare our user's features to.
        last_feature_progressions: The feature progressions dictionary from the previous comparing step
        last_resampled_features: The resampled values from possibly many previous comparings steps
        feature_states: The current feature states
        
    Returns:
        increase_reps (bool): Is true if a repetition was detected
        new_feature_progressions (dict): Resembles the features of interest specification dictionary but has strings inplace for every feature
                                            that indicate the progress the user made concerning said feature.
        new_resampled_features (dict): Resembles the features of interest specification dictionary but has lists inplace for every feature
                                        that contain the resampled values.
    """
    def reset_child_featuers(d):
        for k, v in d.items():
            if isinstance(v, collections.MutableMapping):
                d[k] = reset_child_featuers(v)
            else:
                d[k] = 0
        return d

    exercise_data = spot_info_dict['exercise_data']
    beginning_states = spot_info_dict['exercise_data']['beginning_state_dict']

    increase_reps = True
    in_beginning_state = True
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
                last_resampled_feature_value = float(last_resampled_features[feature_type][k][-1])
                new_resampled_feature_values = compute_resampled_feature_values(feature_value, last_resampled_feature_value, resolution)
                if new_resampled_feature_values:
                    new_resampled_features[feature_type][k] = new_resampled_feature_values
            except (KeyError, TypeError):
                # If we have no resampled feature values yet, set them to the nearest resampled one
                new_resampled_features[feature_type][k] = scale[np.argmin(abs(scale - feature_value))]
            
            # With the beginning state of a feature and the current feature state, we can computer the new feature progression value
            try:
                last_feature_progression = last_feature_progressions[feature_type][k][-1]
            except (KeyError, TypeError):
                # If we have no last feature progression value, set this feature progression to the starting value
                last_feature_progression = 0
            beginning_state = beginning_states[feature_type][k]['feature_state']
            features_state = features_states[feature_type][k]['feature_state']

            if features_state != beginning_state:
                in_beginning_state = False
            new_feature_progression = compute_new_feature_progression(beginning_state, features_state, last_feature_progression)
            new_feature_progressions[feature_type][k] = new_feature_progression
            if new_feature_progression < exercise_data['reference_feature_data'][feature_type][k]['number_of_changes_in_decided_feature_states']:
                increase_reps = False
            elif new_feature_progression > exercise_data['reference_feature_data'][feature_type][k]['number_of_changes_in_decided_feature_states']:
                bad_repetition = True
                
        # If a data type has no updates, remove it again
        if new_feature_progressions[feature_type] == {}:
            del new_feature_progressions[feature_type]
        if new_resampled_features[feature_type] == {}:
            del new_resampled_features[feature_type]

    if in_beginning_state and bad_repetition:
        rp.logerr("bad rep fixed!!!")
        new_feature_progressions = reset_child_featuers(new_feature_progressions)
        increase_reps, bad_repetition, new_feature_progressions, new_resampled_features = compare_high_level_features(spot_info_dict, new_feature_progressions, last_resampled_features, features_states, False)

    if bad_repetition:
        increase_reps = False

    # TODO: Do something safe here, this might not reset all features
    if increase_reps:
        new_feature_progressions = reset_child_featuers(new_feature_progressions)

    return increase_reps, bad_repetition, new_feature_progressions, new_resampled_features


def calculate_reference_pose_mapping(feature_trajectories: dict, exercise_data: dict) -> np.ndarray:
    """Calculate the pose in the reference trajectory that we think our user is most probably in.

    This method measures the similarity between the recent feature_trajectory of a user and the vectors
    inside a hankel matrix of the reference trajectory. Thereby we can compute how likely it is that the
    use is at a certain point in the execution of the exercise of the expert.

    Args:
        feature_trajectories: A dictionary that holds a list of past feature values for every type of feature
                                in every feature category
        exercise data: A dictionary containing metadata around the exercise

    Returns:
        reference_pose: The reference pose that we think the user is in
    """

    def custom_absolute_difference(a, b):
        """Calculate the absolute difference between two values on a "ring" scale between 0 and 1."""
        smaller = min([a, b])
        larger = max([a, b])
        x = abs(larger - smaller)
        y = abs(smaller + 1 - larger)
        return min(x, y)

    # return {"lower_boundary": lower_boundary, \
    #         "upper_boundary": upper_boundary, \
    #             "lowest_value": lowest_value, \
    #                 "highest_value": highest_value, \
    #                     "range_of_motion": range_of_motion, \
    #                         "resampled_values_reference_trajectory_indices": resampled_values_reference_trajectory_indices, \
    #                             "reference_trajectory_hankel_matrices": reference_trajectory_hankel_matrices, \
    #                                 "resampled_reference_trajectory_scale_array": resampled_reference_trajectory_scale_array, \
    #                                     "median_trajectory": median_reference_trajectory, \
    #                                         "median_reference_trajectory_feature_states": median_reference_trajectory_feature_states, \
    #                                             "median_resampled_values_reference_trajectory_fractions": median_resampled_values_reference_trajectory_fractions,
    #                                                 "number_of_changes_in_decided_feature_states": number_of_changes_in_decided_feature_states}

    reference_poses = exercise_data['recording']
    predicted_indices = []
    median_resampled_values_reference_trajectory_fractions = []

    for feature_type, features in exercise_data['reference_feature_data'].items():
        for k, v in features.items():
            resampled_values_reference_trajectory_indices = v['resampled_values_reference_trajectory_indices']
            reference_trajectory_hankel_matrices = v['reference_trajectory_hankel_matrices']
            feature_trajectory = np.asarray(feature_trajectories[feature_type][k], np.float16)

            for idx, reference_trajectory_hankel_matrix in enumerate(reference_trajectory_hankel_matrices):
                errors = custom_metric(reference_trajectory_hankel_matrix, feature_trajectory, 4, 0)
                prediction = np.argmin(errors)
                index = resampled_values_reference_trajectory_indices[idx][prediction]
                median_resampled_values_reference_trajectory_fractions.append(v['median_resampled_values_reference_trajectory_fractions'][prediction])

                predicted_indices.append(index)

    median_resampled_values_reference_trajectory_fractions_errors = []
    # TODO: This is a little bit overkill but should still give the correct result, maybe change to something more elegant
    for idx1, value in enumerate(median_resampled_values_reference_trajectory_fractions):
        for idx2 in range(len(median_resampled_values_reference_trajectory_fractions)):
            if idx2 == idx1:
                continue
            median_resampled_values_reference_trajectory_fractions_errors.append(custom_absolute_difference(value, median_resampled_values_reference_trajectory_fractions[idx2]))
    
    predicted_pose_index = int(np.average(predicted_indices))
    reference_pose = reference_poses[predicted_pose_index]

    mean_resampled_values_reference_trajectory_fractions_average_difference = np.average(median_resampled_values_reference_trajectory_fractions_errors)/2 # divide by two, since we account for every errors twice

    return reference_pose, mean_resampled_values_reference_trajectory_fractions_average_difference


class Comparator(Thread):
    """Pops data from inbound spot queues and calculates and metrics which are put into outbound message queues.
    
    A Comparator thread asks a spot queue load balancer what spot queue to dequeue from.
    User data, i.e. a skelleton, is dequeued and used to compute information like the progress in an exercise or the
    reaching of a repetition. The Comparator can be scaled horizontally, meaning that more threads can be spawned to
    scale the throughput of the Comparator thread.
    """
    def __init__(self,
    spot_key: str,
    spot_metadata_interface_class: SpotMetaDataInterface = RedisSpotMetaDataInterface, 
    spot_queue_interface_class: SpotQueueInterface = RedisSpotQueueInterface,
    feature_extractor_class: FeatureExtractor = SpinFeatureExtractor,
    feature_data_queues_interface_class: PastFeatureDataQueuesInterface = RedisFeatureDataQueuesInterface):
        super(Comparator, self).__init__()

        self.spot_queue_interface = spot_queue_interface_class()
        self.spot_metadata_interface = spot_metadata_interface_class()
        self.feature_extractor = feature_extractor_class()
        # Use the same interface class for these two, because they have the same needs to the interface 
        self.past_features_queue_interface = feature_data_queues_interface_class()
        self.past_resampled_features_queue_interface = feature_data_queues_interface_class()
        
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)

        # This can be set to false by an external entity to stop the loop from running
        self.running = True

        self.user_exercise_state_publisher = rp.Publisher(ROS_TOPIC_USER_EXERCISE_STATES, String, queue_size=1000)  
        self.user_correction_publisher = rp.Publisher(ROS_TOPIC_USER_CORRECTIONS, String, queue_size=1000)  

        self.predicted_skelleton_publisher = rp.Publisher("comparing_reference_prediction", Person, queue_size=1000)
        self.user_skelleton_publisher = rp.Publisher("comparing_input", Person, queue_size=1000)

        self.last_feature_progressions = {}
        self.last_resampled_features = {}

        self.spot_key = spot_key

        self.last_mean_resampled_values_reference_trajectory_fractions_average_differences = []
        self.bad_repetition = False

        self.start()

    @lru_cache(maxsize=EXERCISE_DATA_LRU_CACHE_SIZE)
    def get_exercise_data(self, spot_info_key, exercise_data_hash):
        """Gets data on the exercise that is to be performed on the spot.
        
        This method uses lru caching because exercise information is usually obtained on every step, which makes
        it a very costly operation if it includes fetching data from a stata store (like Redis) and deserializing it.
        """
        return self.spot_metadata_interface.get_spot_info_dict(spot_info_key, ["exercise_data"])

    def run(self) -> NoReturn:
        _, _, spot_info_key, spot_state_key, spot_feature_progression_key, spot_resampled_features_key = generate_redis_key_names(self.spot_key)
        # Fetch last feature progressions and resampled feature lists
        self.last_feature_progressions = self.past_features_queue_interface.get_features(spot_feature_progression_key, latest_only=True)
        self.last_resampled_features = self.past_resampled_features_queue_interface.get_features(spot_resampled_features_key, latest_only=True)
        while(self.running):
            try:
                # The following lines fetch data that we need to compare
                spot_info_dict = self.spot_metadata_interface.get_spot_info_dict(spot_info_key, ["exercise_data_hash", "start_time", "repetitions"])
                spot_info_dict.update(self.get_exercise_data(spot_info_key, spot_info_dict["exercise_data_hash"]))

                # As long as there are skelletons available for this spot, continue
                past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list = self.spot_queue_interface.dequeue(self.spot_key)

                # Extract feature states
                exercise_data = spot_info_dict['exercise_data']
                used_joint_ndarray = joints_with_timestamp['used_joint_ndarray']
                features_states = self.feature_extractor.extract_states(used_joint_ndarray, exercise_data['reference_feature_data'], exercise_data['feature_of_interest_specification'])

                # Compare joints with expert system data
                increase_reps, bad_repetition, new_features_progressions, new_resampled_features = compare_high_level_features(spot_info_dict, self.last_feature_progressions, self.last_resampled_features, features_states, self.bad_repetition)

                self.last_feature_progressions = enqueue_dictionary(self.last_feature_progressions, new_features_progressions)
                self.last_resampled_features = enqueue_dictionary(self.last_resampled_features, new_resampled_features)

                self.bad_repetition = bad_repetition

                # Send info back to REST API
                if increase_reps:
                    spot_info_dict['repetitions'] = int(spot_info_dict['repetitions']) + 1
                    user_state_data = {
                        'user_id': 0,
                        'current_exercise_name': spot_info_dict.get('exercise_data').get('name'),
                        'repetitions': spot_info_dict['repetitions'],
                        'seconds_since_last_exercise_start': (time.time_ns() - int(spot_info_dict.get('start_time'))) / 1e+9,
                        'milliseconds_since_last_repetition': 0,
                        'repetition_score': 100,
                        'exercise_score': 100,
                    }
                    publish_message(self.user_exercise_state_publisher, ROS_TOPIC_USER_EXERCISE_STATES, user_state_data)

                    self.spot_metadata_interface.set_spot_info_dict(spot_info_key, {"repetitions": spot_info_dict['repetitions']})

                # Calculate a new reference pose mapping
                if new_resampled_features:
                    reference_pose, mean_resampled_values_reference_trajectory_fractions_average_difference = calculate_reference_pose_mapping(self.last_resampled_features, spot_info_dict['exercise_data'])
                    self.last_mean_resampled_values_reference_trajectory_fractions_average_differences.append(mean_resampled_values_reference_trajectory_fractions_average_difference)
                    if len(self.last_mean_resampled_values_reference_trajectory_fractions_average_differences) >= FEATURE_DIFFERENCE_MAX_QUEUE_LENGTH:
                        del self.last_mean_resampled_values_reference_trajectory_fractions_average_differences[0]
                    if np.average(self.last_mean_resampled_values_reference_trajectory_fractions_average_differences) >= FEATURE_DIFFERENCE_ELASTICITY:
                        self.bad_repetition = True
                        # rp.logerr("REPETITION NOT COUNTED!")
                        
                    reference_body_parts = self.feature_extractor.ndarray_to_body_parts(reference_pose)
                    reference_person_msg = Person()
                    reference_person_msg.stationID = self.spot_key
                    reference_person_msg.sensorID = -1
                    reference_person_msg.bodyParts = reference_body_parts
                    self.predicted_skelleton_publisher.publish(reference_person_msg)

                    user_body_parts = self.feature_extractor.ndarray_to_body_parts(joints_with_timestamp['used_joint_ndarray'])
                    user_person_msg = Person()
                    user_person_msg.stationID = self.spot_key
                    user_person_msg.sensorID = -1
                    user_person_msg.bodyParts = user_body_parts
                    self.user_skelleton_publisher.publish(user_person_msg)

                # Corrections are not part of the beta release, we therefore leave them out and never send user correction messages
                correction = None

                if correction != None and SEND_CORRETIONS:
                    user_correction_message = {
                        'user_id': 0,
                        'repetition': spot_info_dict['repetitions'],
                        'positive_correction': False,
                        'display_text': correction
                    }
                    publish_message(self.user_correction_publisher, ROS_TOPIC_USER_CORRECTIONS, user_correction_message)
            except QueueEmpty:
                continue
            except SpotMetaDataException as e:
                if HIGH_VERBOSITY:
                    rp.logerr(e)    
            except Exception as e:
                if HIGH_VERBOSITY:
                    print_exc() 
                    rp.logerr("Encountered an Error while Comparing: " + str(e))
            
        # Enqueue data for feature progressions and resampled feature lists
        self.past_features_queue_interface.enqueue(spot_feature_progression_key, self.last_feature_progressions)
        self.past_resampled_features_queue_interface.enqueue(spot_resampled_features_key, self.last_resampled_features)
