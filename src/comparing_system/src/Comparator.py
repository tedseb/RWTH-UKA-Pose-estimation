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

import time
from functools import lru_cache
from importlib import import_module
from threading import Thread
from traceback import print_exc
from typing import NoReturn

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

class MalformedFeatures(Exception):
    pass

def compute_new_feature_progression(beginning_state, features_state, last_feature_progression):
    """Compute a dictionary representing the progression of the features specified by feature_state

    This method turns features states, such as FEATURE_HIGH or FEATURE_LOW into a feature progression,
    such as PROGRESSION_PARTIAL, PROGRESSION_DONE or PROGRESSION_START, depending on the previous progression
    of the feature. This way we can track the progression of different features between timesteps.

    Args: 
        beginning_state: A dictionary that holds the state in which a feature begins for every feature of every category
        features_state: The state that the featuers are in
        last_feature_progression: The last dictionary produced by this method in the last timestep

    Return:
        new_feature_progression: The feature progression dictionary at this timestep

    Raises:
        MalformedFeatures: If features are not the expected form.
    """
    new_feature_progression = last_feature_progression

    # If features beginn with the FEATURE_HIGH state, we need to check if they have passed through the FEATURE_LOW state
    if beginning_state == FEATURE_HIGH:
        if features_state == FEATURE_HIGH:
            feature_is_in_beginning_state = True
        elif features_state == FEATURE_LOW:
            new_feature_progression  = PROGRESSION_PARTIAL
            feature_is_in_beginning_state = False
        else:
            feature_is_in_beginning_state = False
    # If features beginn with the FEATURE_LOW state, we need to check if they have passed through the FEATURE_HIGH state
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
    
    # If features are in the beginning state and they have progressed already, their progression can be set to PROGRESSION_DONE
    if feature_is_in_beginning_state:
        if last_feature_progression in (PROGRESSION_DONE, PROGRESSION_PARTIAL):
            new_feature_progression  = PROGRESSION_DONE
        else:
            new_feature_progression = PROGRESSION_START

    return new_feature_progression


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
    feature_trajectory_shortened = feature_trajectory[:comparing_length]
    distances = hankel_matrix_shortened - feature_trajectory_shortened
    fading_factor = np.linspace(min_weight, max_weight_, comparing_length) # Let older signals have less influence on the error
    errors = np.linalg.norm((distances) * fading_factor, axis=1)
    return errors


def compare_high_level_features(spot_info_dict: dict, 
    last_feature_progressions: dict,
    last_resampled_features: dict,
    features_states: dict) -> Tuple[bool, dict, Any]:
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
    exercise_data = spot_info_dict['exercise_data']
    beginning_states = spot_info_dict['exercise_data']['beginning_state_dict']

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
            new_feature_progression = compute_new_feature_progression(beginning_state, features_state, last_feature_progression)
            
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
    reference_poses = exercise_data['recording']
    predicted_indices = []

    for feature_type, features in exercise_data['reference_feature_data'].items():
        for k, v in features.items():
            resampled_values_reference_trajectory_indices = v['resampled_values_reference_trajectory_indices']
            reference_trajectory_hankel_matrices = v['reference_trajectory_hankel_matrices']
            feature_trajectory = np.asarray(feature_trajectories[feature_type][k], np.float16)

            for idx, reference_trajectory_hankel_matrix in enumerate(reference_trajectory_hankel_matrices):
                errors = custom_metric(reference_trajectory_hankel_matrix, feature_trajectory, 4, 0)
                prediction = np.argmin(errors)
                index = resampled_values_reference_trajectory_indices[idx][prediction]

                predicted_indices.append(index)

    predicted_pose_index = int(np.average(predicted_indices))
    reference_pose = reference_poses[predicted_pose_index]
    
    return reference_pose


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

        self.spot_key = spot_key

        self.start()

    @lru_cache(maxsize=EXERCISE_DATA_LRU_CACHE_SIZE)
    def get_exercise_data(self, spot_info_key, exercise_data_hash):
        """Gets data on the exercise that is to be performed on the spot.
        
        This method uses lru caching because exercise information is usually obtained on every step, which makes
        it a very costly operation if it includes fetching data from a stata store (like Redis) and deserializing it.
        """
        return self.spot_metadata_interface.get_spot_info_dict(spot_info_key, ["exercise_data"])

    def run(self) -> NoReturn:
        while(self.running):
            try:
                # The following lines fetch data that we need to compare
                _, _, spot_info_key, spot_state_key, spot_feature_progression_key, spot_resampled_features_key = generate_redis_key_names(self.spot_key)
                spot_info_dict = self.spot_metadata_interface.get_spot_info_dict(spot_info_key, ["exercise_data_hash", "start_time", "repetitions"])
                spot_info_dict.update(self.get_exercise_data(spot_info_key, spot_info_dict["exercise_data_hash"]))

                # Fetch last feature progressions and resampled feature lists
                last_feature_progressions = self.past_features_queue_interface.get_features(spot_feature_progression_key, latest_only=True)
                last_resampled_features = self.past_resampled_features_queue_interface.get_features(spot_resampled_features_key, latest_only=True)
 
                # As long as there are skelletons available for this spot, continue
                past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list = self.spot_queue_interface.dequeue(self.spot_key)

                # Extract feature states
                exercise_data = spot_info_dict['exercise_data']
                used_joint_ndarray = joints_with_timestamp['used_joint_ndarray']
                features_states = self.feature_extractor.extract_states(used_joint_ndarray, exercise_data['reference_feature_data'], exercise_data['feature_of_interest_specification'])

                # Compare joints with expert system data
                increase_reps, new_features_progression, new_resampled_features = compare_high_level_features(spot_info_dict, last_feature_progressions, last_resampled_features, features_states)

                # Enqueue data for feature progressions and resampled feature lists
                self.past_features_queue_interface.enqueue(spot_feature_progression_key, new_features_progression)
                self.past_resampled_features_queue_interface.enqueue(spot_resampled_features_key, new_resampled_features)

                # Send info back to REST API
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
                    publish_message(self.user_exercise_state_publisher, ROS_TOPIC_USER_EXERCISE_STATES, user_state_message)

                    self.spot_metadata_interface.set_spot_info_dict(spot_info_key, {"repetitions": spot_info_dict['repetitions']})

                # Calculate a new reference pose mapping
                if new_resampled_features:
                    all_resampled_features = self.past_resampled_features_queue_interface.get_features(spot_resampled_features_key, latest_only=False)
                    reference_pose = calculate_reference_pose_mapping(all_resampled_features, spot_info_dict['exercise_data'])
                    reference_body_parts = self.feature_extractor.ndarray_to_body_parts(reference_pose)
                    reference_person_msg = Person()
                    reference_person_msg.stationID = 99
                    reference_person_msg.sensorID = 0
                    reference_person_msg.bodyParts = reference_body_parts
                    self.predicted_skelleton_publisher.publish(reference_person_msg)

                    user_body_parts = self.feature_extractor.ndarray_to_body_parts(joints_with_timestamp['used_joint_ndarray'])
                    user_person_msg = Person()
                    user_person_msg.stationID = 99
                    user_person_msg.sensorID = 0
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
