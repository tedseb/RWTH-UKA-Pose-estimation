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
from scipy import signal

from backend.msg import Person

try:
    from motion_analysis.src.Worker import *
    from motion_analysis.src.InterCom import *
    from motion_analysis.src.DataUtils import *
    from motion_analysis.src.ROSAdapters import *
    from motion_analysis.src.algorithm.GUI import *
    from motion_analysis.src.algorithm.FeatureExtraction import *
    from motion_analysis.src.algorithm.Features import *
    from motion_analysis.src.algorithm.AlgoUtils import *
    from motion_analysis.src.algorithm.Algorithm import *
    from motion_analysis.src.algorithm.logging import log
except ImportError:
    from src.Worker import *
    from src.InterCom import *
    from src.DataUtils import *
    from src.ROSAdapters import *
    from src.algorithm.GUI import *
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
    config: dict, 
    spot_key: str,
    gui: MotionAnaysisGUI = mock.MagicMock(),
    spot_metadata_interface_class: SpotMetaDataInterface = RedisSpotMetaDataInterface, 
    spot_queue_interface_class: SpotQueueInterface = RedisSpotQueueInterface,
    pose_definition_adapter_class: PoseDefinitionAdapter = MetrabsPoseDefinitionAdapter,
    features_interface_interface_class: FeaturesInterface = RedisFeaturesInterface,):
        super().__init__()
        
        self.config = config

        self.spot_queue_interface = spot_queue_interface_class(self.config)
        self.spot_metadata_interface = spot_metadata_interface_class()
        self.pose_definition_adapter = pose_definition_adapter_class()
        # Use the same interface class for these two, because they have the same needs to the interface 
        self.features_interface = features_interface_interface_class()
        
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)

        # This can be set to false by an external entity to stop the loop from running
        self.running = True

        self.user_exercise_state_publisher = rp.Publisher(self.config['ROS_TOPIC_USER_EXERCISE_STATES'], String, queue_size=1000)  
        self.user_correction_publisher = rp.Publisher(self.config['ROS_TOPIC_USER_CORRECTIONS'], String, queue_size=1000)  

        self.predicted_skelleton_publisher = rp.Publisher("motion_analysis_reference_prediction", Person, queue_size=1000)
        self.user_skelleton_publisher = rp.Publisher("motion_analysis_input", Person, queue_size=1000)

        self.spot_key = spot_key

        self.features = {}

        self.bad_repetition = False
        self.moving_average_joint_difference = 0

        self.gui = gui

        self.spot_info_dict = None

        self.skelleton_deltas_since_rep_start = []

        self.start()

    @lru_cache(maxsize=1000)
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
        publisher.publish(msg)

    def run(self) -> NoReturn:
        _, _, spot_info_key, spot_feature_key = generate_redis_key_names(self.spot_key, self.config)
        # Fetch last feature data
        self.features = self.features_interface.get(spot_feature_key)

        self.progress = 0
        self.alignments_this_rep = np.array([])
        self.progress_differences_this_rep = np.array([])
        self.beginning_of_next_repetition_detected = False

        # The following lines fetch data that we need to analyse
        self.spot_info_dict = self.spot_metadata_interface.get_spot_info_dict(spot_info_key, ["exercise_data_hash", "start_time", "repetitions", "station_usage_hash"])

        while(self.running):
            try:
                self.spot_info_dict.update(self.spot_metadata_interface.get_spot_info_dict(spot_info_key, ["exercise_data_hash", "start_time", "station_usage_hash"]))
                self.spot_info_dict.update(self.get_exercise_data(spot_info_key, self.spot_info_dict["exercise_data_hash"]))

                # As long as there are skelletons available for this spot, continue
                past_joints_with_timestamp_list, present_joints_with_timestamp, future_joints_with_timestamp_list = self.spot_queue_interface.dequeue(self.spot_key)

                # Extract feature states
                pose = present_joints_with_timestamp['used_joint_ndarray']

                pose = self.pose_definition_adapter.normalize_skelleton(pose)

                for f in self.features.values():
                    f.update(pose, self.pose_definition_adapter)

                # Compare joints with expert system data
                increase_reps = self.analyze_feature_progressions(self.features)

                # See if progress jumped around too much
                if increase_reps:
                    negative_progress_differences = self.progress_differences_this_rep[np.where(self.progress_differences_this_rep < 0)]
                    positive_progresses_differences = self.progress_differences_this_rep[np.where(self.progress_differences_this_rep > 0)]
                    if len(negative_progress_differences) > 0:
                        neg_mean = abs(np.mean(negative_progress_differences))
                    else:
                        neg_mean = 0
                    if len(positive_progresses_differences) > 0: 
                        pos_median = abs(np.median(positive_progresses_differences))
                    else:
                        pos_median = 0
                    if  neg_mean > pos_median * self.config['JUMPY_PROGRESS_ALPHA'] and self.config['JUMPY_PROGRESS_BETA'] * len(negative_progress_differences) > len(positive_progresses_differences) and not self.config['ROBUST_COUNTING_MODE']:
                        log("Progression was too jumpy in this repetition. Marking this repetition as bad...")
                        self.bad_repetition = True
                        increase_reps = False

                # If the feature alignment in this repetitions is too high, we do not count the repetition
                if increase_reps and np.mean(self.alignments_this_rep) < MINIMAL_ALLOWED_MEAN_FEATURE_ALIGNMENT:
                    log("Feature missalignment during this repetition. Repetition falsified.")
                    increase_reps = False
                    self.alignments_this_rep = np.array([])
                
                # Calculate a new reference pose mapping
                reference_pose = self.calculate_reference_pose_mapping()

                delta = (pose - reference_pose).sum()
                self.skelleton_deltas_since_rep_start.append(delta)
                score = self.calculate_repetition_score()

                # Send info back to REST API
                if increase_reps:
                    score = self.calculate_repetition_score()
                    self.spot_info_dict['repetitions'] = int(self.spot_info_dict['repetitions']) + 1
                    user_state_data = {
                        'station_id': self.spot_key,
                        'current_exercise_name': self.spot_info_dict.get('exercise_data').get('name'),
                        'repetitions': self.spot_info_dict['repetitions'],
                        'miliseconds_since_last_exercise_start': (time.time_ns() - int(self.spot_info_dict.get('start_time', 0))) / 1e+6,
                        'milliseconds_since_last_repetition': 0,
                        'repetition_score': int(score),
                        'exercise_score': 100,
                        'station_usage_hash': self.spot_info_dict.get('station_usage_hash', "")
                    }
                    publish_message(self.user_exercise_state_publisher, self.config['ROS_TOPIC_USER_EXERCISE_STATES'], user_state_data)

                # Publish poses on ROS
                self.publish_pose(reference_pose, self.predicted_skelleton_publisher)
                self.publish_pose(pose, self.user_skelleton_publisher)

                # Corrections are not part of the beta release, we therefore leave them out and never send user correction messages
                correction = None

                if correction != None and SEND_CORRETIONS:
                    user_correction_message = {
                        'user_id': 0,
                        'repetition': self.spot_info_dict['repetitions'],
                        'positive_correction': False,
                        'display_text': correction,
                        'station_usage_hash': self.spot_info_dict.get('station_usage_hash', 0)
                    }
                    publish_message(self.user_correction_publisher, ROS_TOPIC_USER_CORRECTIONS, user_correction_message)
                    
            except QueueEmpty:
                continue
            except Exception as e:
                log_throttle("Encountered an Error while analyzing.")
                log(format_exc())
            
        # Enqueue data for feature progressions and resampled feature lists
        self.features_interface.set(self.spot_key, self.features)
        self.spot_metadata_interface.set_spot_info_dict(spot_info_key, self.spot_info_dict)
        
        self.spot_info_dict = None
        self.features = {}
        self.bad_repetition = False
        self.moving_average_joint_difference = 0

    def calculate_reference_pose_mapping(self) -> np.ndarray:
        """Calculate the pose in the reference trajectory that we think our user is most probably in.

        This method measures the similarity between the recent feature_trajectory of a user and the vectors
        inside a hankel matrix of the reference trajectory. Thereby we can compute how likely it is that the
        use is at a certain point in the execution of the exercise of the expert.

        Returns:
            reference_pose: The reference pose that we think the user is in
            mean_resampled_values_reference_trajectory_fractions_average_difference: The average difference of "where different features think we are"

        """
        recordings = self.spot_info_dict['exercise_data']['recordings']

        if len(recordings.values()) > 1:
            raise NotImplementedError("We have not gotten this method ready for multiple recordings!")

        recording = list(recordings.values())[0]

        predicted_indices = []
        median_resampled_values_reference_trajectory_fractions = []
        progress_vectors = []


        for h, f in self.features.items():
            # For our algorithm, we compare the discretized trajectories of our reference trajectories and our user's trajectory
            discretization_reference_trajectory_indices_tensor = f.reference_feature_collection.discretization_reference_trajectory_indices_tensor
            hankel_tensor_2 = f.reference_feature_collection.hankel_tensor
            discrete_feature_trajectory = np.array(f.discretized_values)

            for idx, hankel_tensor_1 in enumerate(hankel_tensor_2):
                errors = trajectory_distance(hankel_tensor_1, discrete_feature_trajectory, 100, 1)
                prediction = np.argmin(errors)
                index = discretization_reference_trajectory_indices_tensor[idx][prediction]
                median_resampled_values_reference_trajectory_fraction_dict = f.reference_feature_collection.median_trajectory_discretization_ranges[prediction]
                feature_progress = np.mean([median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_from"], median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_to"]])
                progress_vector = map_progress_to_vector(feature_progress)
                progress_vectors.append(progress_vector)
                median_resampled_values_reference_trajectory_fractions.append(median_resampled_values_reference_trajectory_fraction_dict)
                predicted_indices.append(index)

                f.errors = errors
                f.progress_vector = progress_vector
                f.prediction = prediction

                update_gui_features(self.gui, f)
                    
            # Look at every reference feature separately
            # for r in f.reference_feature_collection.reference_recording_features:
            #     joint_difference = total_joint_difference(pose, reference_pose)
            #     r.moving_average_joint_difference = r.moving_average_joint_difference * JOINT_DIFFERENCE_FADING_FACTOR + r * (1 - JOINT_DIFFERENCE_FADING_FACTOR)

        last_progress = self.progress

        self.progress, self.alignment, self.progress_alignment_vector = map_vectors_to_progress_and_alignment(vectors=progress_vectors)

        self.alignments_this_rep = np.append(self.alignments_this_rep, self.alignment)
        self.progress_differences_this_rep = np.append(self.progress_differences_this_rep, self.progress - last_progress)

        update_gui_progress(self.gui, self.progress, self.alignment, self.progress_alignment_vector)

        reference_pose = recording[int(len(recording) * self.progress)]

        return reference_pose

    def analyze_feature_progressions(self,
        features: dict) -> Tuple[bool, bool, dict, Any]:
        """Detect done and bad repetitions by analyzing the feature's progressions.
        
        Args:
            features: The feature dictionary from the previous analysis step
            
        Returns:
            increase_reps (bool): Is true if a repetition was detected
        """
        increase_reps = True
        in_beginning_state = True
        num_features_in_beginning_state = 0
        num_features_progressed = 0

        for f in features.values():
            beginning_state = f.reference_feature_collection.median_beginning_state
            number_of_dicided_state_changes_for_repetition = f.reference_feature_collection.number_of_dicided_state_changes
            if f.state != beginning_state:
                num_features_progressed += 1
            if f.progression < number_of_dicided_state_changes_for_repetition:
                increase_reps = False
            elif f.progression > number_of_dicided_state_changes_for_repetition and not self.config['ROBUST_COUNTING_MODE']:
                log("A feature has progressed through too many states. Marking this repetition as bad...")
                log("Feature specification: " + str(f.specification_dict))
                self.bad_repetition = True
                num_features_progressed += 1
            elif f.progression == number_of_dicided_state_changes_for_repetition:
                num_features_progressed += 1

        if num_features_progressed < num_features_in_beginning_state: # TODO: This is a parameter to be tuned!!
            increase_reps = False


        # Look at every reference feature separately
        # bad_repetition_yes = 0
        # bad_repetition_no = 0
        # increase_reps_yes = 0
        # increase_reps_no = 0
        # in_beginning_state_yes = 0
        # in_beginning_state_no = 0

        # for f in features.values():
        #     for r in f.reference_feature_collection.reference_features:
                
        #         beginning_state = r.median_beginning_state
        #         number_of_dicided_state_changes_for_repetition = r.number_of_dicided_state_changes

        #         if f.state != beginning_state:
        #             in_beginning_state_no += 1
        #         else:
        #             in_beginning_state_yes += 1

        #         if f.progression < number_of_dicided_state_changes_for_repetition:
        #             increase_reps_no += 1
        #         elif f.progression > number_of_dicided_state_changes_for_repetition:
        #             bad_repetition = True
        #             bad_repetition_yes += 1
        #             increase_reps_no += 1
        #         else:
        #             bad_repetition_no += 1
        #             increase_reps_yes += 1

        # bad_repetition_ratio = bad_repetition_yes/bad_repetition_no
        # increase_reps_ratio = increase_reps_yes/increase_reps_no
        # in_beginning_state_ratio = in_beginning_state_yes/in_beginning_state_no


        if in_beginning_state and self.bad_repetition:
            log("Bad repetition aborted. Resetting feature progressions and repetition data...")
            for f in features.values():
                f.progression = 0
            self.bad_repetition = False
            self.beginning_of_next_repetition_detected = True
            
            increase_reps = self.analyze_feature_progressions(features)

        if self.bad_repetition and not self.config['ROBUST_COUNTING_MODE']:
            increase_reps = False

        if increase_reps:
            log("All features have progressed. Repetition detected. Resetting feature progressions...")
            for f in features.values():
                f.progression = 0
            self.beginning_of_next_repetition_detected = True

        if not in_beginning_state and self.beginning_of_next_repetition_detected:
            log("Last repetition started and ended, measuring feature alignment and progress differences for new repetition...")
            self.skelleton_deltas_since_rep_start = []
            self.beginning_of_next_repetition_detected = False
            self.alignments_this_rep = np.array([])
            self.progress_differences_this_rep = np.array([])
            self.progress = 0


        return increase_reps

    def calculate_repetition_score(self):
        x = self.skelleton_deltas_since_rep_start
        alpha = 1/20 # This has to be heuristically determined, as is depends on the size of the normal skelleton
        critical_freq = 0.01 # TODO: Choose heuristically!!
        # Filter scores in order to minimize impact of faulty reference skelleton predictions
        b, a = signal.butter(4, critical_freq, analog=False)
        x = signal.filtfilt(b, a, x)
        average_delta = np.average(x) # Try average and median here, see which one works better
        delta_score = np.absolute(average_delta * alpha)
        score = 100 * np.clip(1 - delta_score, 0, 1)
        return score