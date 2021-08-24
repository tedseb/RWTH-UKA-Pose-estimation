#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Worker Thread.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
The Node spawns mutiple Worker threads.
A Worker thread dequeues data from spot queues, extracts features and compares them to exercises performed
by experts, i.e. data from the expert system.
Whatever information is gained is sent via outgoing message queues.
"""

import time
from functools import lru_cache
from threading import Thread
from traceback import print_exc
from typing import NoReturn

import redis
import numpy as np
import rospy as rp

try:
    from motion_analysis.src.Worker import *
    from motion_analysis.src.DataConfig import *
    from motion_analysis.src.InterCom import *
    from motion_analysis.src.DataUtils import *
    from motion_analysis.src.ROSAdapters import *
    from motion_analysis.src.algorithm.AlgoConfig import *
    from motion_analysis.src.algorithm.FeatureExtraction import *
    from motion_analysis.src.algorithm.AlgoUtils import *
except ImportError:
    from src.Worker import *
    from src.DataConfig import *
    from src.InterCom import *
    from src.DataUtils import *
    from src.ROSAdapters import *
    from src.algorithm.AlgoConfig import *
    from src.algorithm.FeatureExtraction import *
    from src.algorithm.AlgoUtils import *


class NoJointsAvailable(Exception):
    pass

class NoSpoMetaDataAvailable(Exception):
    pass

class NoExerciseDataAvailable(Exception):
    pass

class Worker(Thread):
    """Pops data from inbound spot queues and calculates and metrics which are put into outbound message queues.
    
    A Worker thread asks a spot queue load balancer what spot queue to dequeue from.
    User data, i.e. a skelleton, is dequeued and used to compute information like the progress in an exercise or the
    reaching of a repetition. The Worker can be scaled horizontally, meaning that more threads can be spawned to
    scale the throughput of the Worker thread.
    """
    def __init__(self,
    spot_key: str,
    spot_metadata_interface_class: SpotMetaDataInterface = RedisSpotMetaDataInterface, 
    spot_queue_interface_class: SpotQueueInterface = RedisSpotQueueInterface,
    pose_definition_adapter_class: PoseDefinitionAdapter = SpinPoseDefinitionAdapter,
    feature_data_queues_interface_class: PastFeatureDataQueuesInterface = RedisFeatureDataQueuesInterface):
        super(Worker, self).__init__()

        self.spot_queue_interface = spot_queue_interface_class()
        self.spot_metadata_interface = spot_metadata_interface_class()
        self.pose_definition_adapter = pose_definition_adapter_class()
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
        self.last_feature_states = {}

        self.spot_key = spot_key

        self.last_mean_resampled_values_reference_trajectory_fractions_average_differences = []
        self.bad_repetition = False

        self.last_used_joint_ndarray = []

        self.start()

    @lru_cache(maxsize=EXERCISE_DATA_LRU_CACHE_SIZE)
    def get_exercise_data(self, spot_info_key, exercise_data_hash):
        """Gets data on the exercise that is to be performed on the spot.
        
        This method uses lru caching because exercise information is usually obtained on every step, which makes
        it a very costly operation if it includes fetching data from a stata store (like Redis) and deserializing it.
        """
        # rp.logerr("Uncached data fetched")
        self.last_feature_progressions = {}
        self.last_resampled_features = {}
        self.last_feature_states = {}
        
        return self.spot_metadata_interface.get_spot_info_dict(spot_info_key, ["exercise_data"])

    def run(self) -> NoReturn:
        _, _, spot_info_key, spot_state_key, spot_feature_progression_key, spot_resampled_features_key = generate_redis_key_names(self.spot_key)
        # Fetch last feature progressions and resampled feature lists
        self.last_feature_progressions = self.past_features_queue_interface.get_features(spot_feature_progression_key, latest_only=True)
        self.last_resampled_features = self.past_resampled_features_queue_interface.get_features(spot_resampled_features_key, latest_only=True)
        while(self.running):
            try:
                # The following lines fetch data that we need to analyse
                spot_info_dict = self.spot_metadata_interface.get_spot_info_dict(spot_info_key, ["exercise_data_hash", "start_time", "repetitions"])
                spot_info_dict.update(self.get_exercise_data(spot_info_key, spot_info_dict["exercise_data_hash"]))

                # As long as there are skelletons available for this spot, continue
                past_joints_with_timestamp_list, present_joints_with_timestamp, future_joints_with_timestamp_list = self.spot_queue_interface.dequeue(self.spot_key)

                self.last_used_joint_ndarray.append(present_joints_with_timestamp['used_joint_ndarray'])

                if len(self.last_used_joint_ndarray) > AVERAGE_SKELETON_QUEUE_LENGTH:
                    self.last_used_joint_ndarray.pop(0)

                # Extract feature states
                exercise_data = spot_info_dict['exercise_data']
                used_joint_ndarray = np.mean(self.last_used_joint_ndarray, axis=0)
                features_states = extract_states(used_joint_ndarray, self.last_feature_states, exercise_data['reference_feature_data'], exercise_data['feature_of_interest_specification'], self.pose_definition_adapter)

                # rp.logerr(used_joint_ndarray)

                self.last_feature_states = features_states

                # Compare joints with expert system data
                increase_reps, bad_repetition, new_features_progressions, new_resampled_features = compare_high_level_features(spot_info_dict, self.last_feature_progressions, self.last_resampled_features, features_states, self.bad_repetition)

                # if self.bad_repetition != bad_repetition:
                    # rp.logerr(self.bad_repetition)
                    # rp.logerr(bad_repetition)
                    # rp.logerr(features_states)
                    # rp.logerr(new_features_progressions)
                    # rp.logerr(self.last_resampled_features)
                    
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
                        if not MESSY_INPUTS:
                            self.bad_repetition = True

                            self.last_feature_progressions = {}
                            self.last_resampled_features = {}
                            self.last_feature_states = {}
                            rp.logerr("bad_repetition detected : FEATURE_DIFFERENCE_ELASTICITY")
                        
                    reference_body_parts = self.pose_definition_adapter.ndarray_to_body_parts(reference_pose)
                    reference_person_msg = Person()
                    reference_person_msg.stationID = self.spot_key
                    reference_person_msg.sensorID = -1
                    reference_person_msg.bodyParts = reference_body_parts
                    self.predicted_skelleton_publisher.publish(reference_person_msg)

                    user_body_parts = self.pose_definition_adapter.ndarray_to_body_parts(used_joint_ndarray)
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
