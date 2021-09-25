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
from traceback import format_exc
from typing import NoReturn

import redis
import numpy as np
import rospy as rp
from std_msgs.msg import String
from unittest import mock

from backend.msg import Person

try:
    from motion_analysis.src.Worker import *
    from motion_analysis.src.DataConfig import *
    from motion_analysis.src.InterCom import *
    from motion_analysis.src.DataUtils import *
    from motion_analysis.src.ROSAdapters import *
    from motion_analysis.src.algorithm.GUI import *
    from motion_analysis.src.algorithm.AlgoConfig import *
    from motion_analysis.src.algorithm.FeatureExtraction import *
    from motion_analysis.src.algorithm.Features import *
    from motion_analysis.src.algorithm.AlgoUtils import *
    from motion_analysis.src.algorithm.Algorithm import *
    from motion_analysis.src.algorithm.logging import log
except ImportError:
    from src.Worker import *
    from src.DataConfig import *
    from src.InterCom import *
    from src.DataUtils import *
    from src.ROSAdapters import *
    from src.algorithm.GUI import *
    from src.algorithm.AlgoConfig import *
    from src.algorithm.FeatureExtraction import *
    from src.algorithm.Features import *
    from src.algorithm.AlgoUtils import *
    from src.algorithm.Algorithm import *
    from src.algorithm.logging import log


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
    gui: MotionAnaysisGUI = mock.MagicMock(),
    spot_metadata_interface_class: SpotMetaDataInterface = RedisSpotMetaDataInterface, 
    spot_queue_interface_class: SpotQueueInterface = RedisSpotQueueInterface,
    pose_definition_adapter_class: PoseDefinitionAdapter = SpinPoseDefinitionAdapter,
    features_interface_interface_class: FeaturesInterface = RedisFeaturesInterface,):
        super().__init__()

        self.spot_queue_interface = spot_queue_interface_class()
        self.spot_metadata_interface = spot_metadata_interface_class()
        self.pose_definition_adapter = pose_definition_adapter_class()
        # Use the same interface class for these two, because they have the same needs to the interface 
        self.features_interface = features_interface_interface_class()
        
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)

        # This can be set to false by an external entity to stop the loop from running
        self.running = True

        self.user_exercise_state_publisher = rp.Publisher(ROS_TOPIC_USER_EXERCISE_STATES, String, queue_size=1000)  
        self.user_correction_publisher = rp.Publisher(ROS_TOPIC_USER_CORRECTIONS, String, queue_size=1000)  

        self.predicted_skelleton_publisher = rp.Publisher("comparing_reference_prediction", Person, queue_size=1000)
        self.user_skelleton_publisher = rp.Publisher("comparing_input", Person, queue_size=1000)

        self.spot_key = spot_key

        self.features = {}

        self.last_mean_resampled_values_reference_trajectory_fractions_average_differences = []

        self.bad_repetition = False
        self.moving_average_joint_difference = 0

        self.gui = gui

        self.spot_info_dict = None

        self.start()

    @lru_cache(maxsize=EXERCISE_DATA_LRU_CACHE_SIZE)
    def get_exercise_data(self, spot_info_key, exercise_data_hash):
        """Gets data on the exercise that is to be performed on the spot.
        
        This method uses lru caching because exercise information is usually obtained on every step, which makes
        it a very costly operation if it includes fetching data from a stata store (like Redis) and deserializing it.
        """
        return self.spot_metadata_interface.get_spot_info_dict(spot_info_key, ["exercise_data"])

    def publish_pose(self, pose: np.ndarray, publisher):
        ros_pose = self.pose_definition_adapter.ndarray_to_body_parts(pose)
        msg = Person()
        msg.stationID = self.spot_key
        msg.sensorID = -1
        msg.bodyParts = ros_pose
        self.predicted_skelleton_publisher.publish(msg)

    def run(self) -> NoReturn:
        _, _, spot_info_key, spot_feature_key = generate_redis_key_names(self.spot_key)
        # Fetch last feature data
        self.features = self.features_interface.get(spot_feature_key)

        # The following lines fetch data that we need to analyse
        self.spot_info_dict = self.spot_metadata_interface.get_spot_info_dict(spot_info_key, ["exercise_data_hash", "start_time", "repetitions"])

        while(self.running):
            try:
                self.spot_info_dict.update(self.get_exercise_data(spot_info_key, self.spot_info_dict["exercise_data_hash"]))

                # As long as there are skelletons available for this spot, continue
                past_joints_with_timestamp_list, present_joints_with_timestamp, future_joints_with_timestamp_list = self.spot_queue_interface.dequeue(self.spot_key)

                # Extract feature states
                pose = present_joints_with_timestamp['used_joint_ndarray']

                for f in self.features.values():
                    f.update(pose, self.pose_definition_adapter)

                # Compare joints with expert system data
                increase_reps, self.bad_repetition = analyze_feature_progressions(self.features, self.bad_repetition)

                # Send info back to REST API
                if increase_reps:
                    self.spot_info_dict['repetitions'] = int(self.spot_info_dict['repetitions']) + 1
                    user_state_data = {
                        'user_id': 0,
                        'current_exercise_name': self.spot_info_dict.get('exercise_data').get('name'),
                        'repetitions': self.spot_info_dict['repetitions'],
                        'seconds_since_last_exercise_start': (time.time_ns() - int(self.spot_info_dict.get('start_time'))) / 1e+9,
                        'milliseconds_since_last_repetition': 0,
                        'repetition_score': 100,
                        'exercise_score': 100,
                    }
                    publish_message(self.user_exercise_state_publisher, ROS_TOPIC_USER_EXERCISE_STATES, user_state_data)

                # Calculate a new reference pose mapping
                # TODO: make this pretty
                reference_pose, mean_resampled_values_reference_trajectory_fractions_average_difference = calculate_reference_pose_mapping(self.features, self.spot_info_dict['exercise_data'], self.gui)
                self.last_mean_resampled_values_reference_trajectory_fractions_average_differences.append(mean_resampled_values_reference_trajectory_fractions_average_difference)
                if len(self.last_mean_resampled_values_reference_trajectory_fractions_average_differences) >= FEATURE_DIFFERENCE_MAX_QUEUE_LENGTH:
                    del self.last_mean_resampled_values_reference_trajectory_fractions_average_differences[0]
                if np.average(self.last_mean_resampled_values_reference_trajectory_fractions_average_differences) >= FEATURE_DIFFERENCE_ELASTICITY:
                    if not MESSY_INPUTS:
                        # Use this only if AI produces adequate results
                        self.bad_repetition = True

                self.publish_pose(reference_pose, self.predicted_skelleton_publisher)
                self.publish_pose(pose, self.user_skelleton_publisher)

                # Corrections are not part of the beta release, we therefore leave them out and never send user correction messages
                correction = None

                if correction != None and SEND_CORRETIONS:
                    user_correction_message = {
                        'user_id': 0,
                        'repetition': self.spot_info_dict['repetitions'],
                        'positive_correction': False,
                        'display_text': correction
                    }
                    publish_message(self.user_correction_publisher, ROS_TOPIC_USER_CORRECTIONS, user_correction_message)
                    
            except QueueEmpty:
                continue
            except Exception as e:
                log_throttle("Encountered an Error while Comparing")
                log(format_exc())
            
        # Enqueue data for feature progressions and resampled feature lists
        self.features_interface.set(self.spot_key, self.features)
        self.spot_metadata_interface.set_spot_info_dict(spot_info_key, self.spot_info_dict)
        
        self.spot_info_dict = None
        self.features = {}
        self.last_mean_resampled_values_reference_trajectory_fractions_average_differences = []
        self.bad_repetition = False
        self.moving_average_joint_difference = 0
