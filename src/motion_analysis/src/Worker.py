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
from typing import NoReturn, Dict

import redis
import numpy as np
import rospy as rp
from std_msgs.msg import String, Int16
from unittest import mock
from scipy import signal
import logy
import pandas as pd

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
        
        self.running = True

        self.user_exercise_state_publisher = rp.Publisher(self.config['ROS_TOPIC_USER_EXERCISE_STATES'], String, queue_size=1000)  
        self.user_correction_publisher = rp.Publisher(self.config['ROS_TOPIC_USER_CORRECTIONS'], String, queue_size=1000)

        self.showroom_reference_progress_publisher = rp.Publisher(self.config['SHOW_ROOM_REFERENCE_PROGRESS_TOPIC'], Int16, queue_size=1000)
        self.showroom_reference_frame_time_publisher =  rp.Publisher(self.config['SHOW_ROOM_REFERENCE_FRAME_TIME_TOPIC'], Int16, queue_size=1000)

        self.predicted_skelleton_publisher = rp.Publisher("motion_analysis_reference_prediction", Person, queue_size=1000)
        self.user_skelleton_publisher = rp.Publisher("motion_analysis_input", Person, queue_size=1000)

        self.spot_key = spot_key

        self.features = {}

        self.bad_repetition_dict = {}

        self.gui = gui

        self.last_score = 0

        self.t = None
        self.progress_velocity = 0

        self.spot_info_dict = None

        self.skelleton_deltas_since_rep_start = []

        self.start()
    
    def log_with_metadata(self, logger, msg) -> NoReturn:
        """ Helper function to log stuff under reference of the spot key and exercise name """
        logger("Spot:" + str(self.spot_key) + ": ExerciseName: " + str(self.spot_info_dict['exercise_data']['name']) + ": " + str(msg))

    @lru_cache(maxsize=1000)
    def get_exercise_data(self, spot_info_key, exercise_data_hash) -> Dict:
        """Gets data on the exercise that is to be performed on the spot.
        
        This method uses lru caching because exercise information is usually obtained on every step, which makes
        it a very costly operation if it includes fetching data from a stata store (like Redis) and deserializing it.
        """
        return self.spot_metadata_interface.get_spot_info_dict(spot_info_key, ["exercise_data"])

    def publish_pose(self, pose: np.ndarray, publisher):
        """ Publish a pose as a BodyParts object in ROS. """
        msg = Person()
        msg.stationID = self.spot_key
        msg.sensorID = -1
        msg.bodyParts = self.pose_definition_adapter.ndarray_to_body_parts(pose)
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
                with logy.TraceTime("motion_analysis_full_algorithm_iter_t"):
                    self.spot_info_dict.update(self.spot_metadata_interface.get_spot_info_dict(spot_info_key, ["exercise_data_hash", "start_time", "station_usage_hash"]))
                    self.spot_info_dict.update(self.get_exercise_data(spot_info_key, self.spot_info_dict["exercise_data_hash"]))

                    # As long as there are skelletons available for this spot, continue
                    past_joints_with_timestamp_list, present_joints_with_timestamp, future_joints_with_timestamp_list = self.spot_queue_interface.dequeue(self.spot_key)

                    # Extract feature states
                    pose = present_joints_with_timestamp['used_joint_ndarray']
                    self.last_t = self.t if self.t else present_joints_with_timestamp['ros_timestamp']
                    self.t = present_joints_with_timestamp['ros_timestamp']

                    pose = self.pose_definition_adapter.normalize_skelleton(pose)

                    for f in self.features.values():
                        f.update_pose(pose, self.pose_definition_adapter)
                        for rf in f.reference_recording_features:
                            rf.update_pose(pose, self.pose_definition_adapter)

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
                        # We do not want negative and positive progresses to relate to each other as follows: (If they do, we mark this repetition as bad)
                        if  neg_mean > pos_median * self.config['JUMPY_PROGRESS_ALPHA'] and self.config['JUMPY_PROGRESS_BETA'] * len(negative_progress_differences) > len(positive_progresses_differences) and self.config ['ENABLE_NUM_FEATURES_TO_PROGRESS_CHECK']:
                            logy.debug("Progression was too jumpy in this repetition. Marking this repetition as bad...")
                            for k in self.bad_repetition_dict.keys():
                                self.bad_repetition_dict[k] = True
                            increase_reps = False
                            self.alignments_this_rep = np.array([])
                        
                    # If the feature alignment in this repetitions is too high, we do not count the repetition
                    if increase_reps and np.mean(self.alignments_this_rep) < self.config['MINIMAL_ALLOWED_MEAN_FEATURE_ALIGNMENT'] and self.config['ENABLE_FEATURE_ALIGNMENT_CHECK']:
                        logy.debug("Feature missalignment during this repetition. Repetition falsified.")
                        increase_reps = False
                        self.alignments_this_rep = np.array([])
                    
                    # Calculate a new reference pose mapping
                    reference_pose, delta = self.calculate_reference_pose_mapping(pose)
                    self.calculate_progress()

                    self.skelleton_deltas_since_rep_start.append(delta)
                    score = self.calculate_repetition_score()

                    update_gui_progress(self.gui, self.progress, self.alignment, self.progress_alignment_vector, score, self.last_score)

                    # Send info back to App
                    if increase_reps:
                        self.last_score = score
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

                    # Publish poses on ROS to observe in RVIZ
                    self.publish_pose(reference_pose, self.predicted_skelleton_publisher)
                    self.publish_pose(pose, self.user_skelleton_publisher)

                    # Corrections are not part of the beta release, we therefore leave them out and never send user correction messages
                    correction = None

                    if correction != None and self.config['SEND_CORRETIONS']:
                        user_correction_message = {
                            'user_id': 0,
                            'repetition': self.spot_info_dict['repetitions'],
                            'positive_correction': False,
                            'display_text': correction,
                            'station_usage_hash': self.spot_info_dict.get('station_usage_hash', 0)
                        }
                        publish_message(self.user_correction_publisher, self.config['ROS_TOPIC_USER_CORRECTIONS'], user_correction_message)
                        
            except QueueEmpty:
                continue
            except Exception as e:
                logy.error_throttle("Encountered an error during motion analysis :" + format_exc(), throttel_time_ms=3000)
            
        # Enqueue data for feature progressions and resampled feature lists
        self.features_interface.set(self.spot_key, self.features)
        self.spot_metadata_interface.set_spot_info_dict(spot_info_key, self.spot_info_dict)
        
        self.spot_info_dict = None
        self.features = {}
        self.bad_repetition = False

    @logy.trace_time("calculate_reference_pose_mapping", period=100)
    def calculate_reference_pose_mapping(self, pose) -> np.ndarray:
        """Calculate the pose in the reference trajectory that we think our user is most probably in.

        This method measures the similarity between the recent feature_trajectory of a user and the vectors
        inside separate hankel matrices per recording.

        Returns:
            pose: The reference pose that we think the user is in
            smallest_delta: The smallest delta we could find, i.e. how good is the pose of our user

        """
        recordings = self.spot_info_dict['exercise_data']['recordings']

        predicted_indices = [[] for h in recordings]
        median_resampled_values_reference_trajectory_fractions = [[] for h in recordings]
        progress_vectors = [[] for h in recordings]

        for h, f in self.features.items():
            # For our algorithm, we compare the discretized trajectories of our reference trajectories and our user's trajectory
            discretization_reference_trajectory_indices_tensor = f.discretization_reference_trajectory_indices_tensor
            hankel_tensor_2 = f.hankel_tensor
            discrete_feature_trajectory = np.array(f.discretized_values)

            for recording_idx, hankel_tensor_1 in enumerate(hankel_tensor_2):
                errors = trajectory_distance(hankel_tensor_1, discrete_feature_trajectory, 100, 1)
                prediction = np.argmin(errors)
                index = discretization_reference_trajectory_indices_tensor[recording_idx][prediction]
                median_resampled_values_reference_trajectory_fraction_dict = f.reference_recording_features[recording_idx].median_trajectory_discretization_ranges[prediction]
                feature_progress = np.mean([median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_from"], median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_to"]])
                progress_vector = map_progress_to_vector(feature_progress)
                progress_vectors[recording_idx].append(progress_vector)
                median_resampled_values_reference_trajectory_fractions[recording_idx].append(median_resampled_values_reference_trajectory_fraction_dict)
                predicted_indices[recording_idx].append(index)

            # For now, these are only for visualization and se we simply set the errors in the feature to the errors for the last recording. Our GUI does not permit anything else
            f.errors = errors
            f.progress_vector = progress_vector
            f.prediction = prediction

            update_gui_features(self.gui, f)

        smallest_delta = np.inf
        best_reference_pose = None
        best_reference_frame_index = 0

        # For a set of progress vectors (one set per recording), we look for the one that leads us to a minimal delta, i.e. the best expert pose
        for idx, vectors in enumerate(progress_vectors):
            recording = recordings[idx]
            progress_estimate, self.alignment, self.progress_alignment_vector = map_vectors_to_progress_and_alignment(vectors=vectors)
            self.alignments_this_rep = np.append(self.alignments_this_rep, self.alignment)
            reference_frame_index = int(len(recording) * progress_estimate)
            reference_pose = recording[reference_frame_index]

            # We calculate the differences between elements of our skelleton
            # TODO: Next step: Calculate difference between elements that we put emphasis on in expert system! (Like certain angles or joints!)
            delta = self.pose_definition_adapter.pose_delta(pose, reference_pose)

            if delta < smallest_delta:
                smallest_delta = delta
                best_reference_pose = reference_pose
                best_reference_frame_index = reference_frame_index
                best_reference_recording_idx = idx
        
        self.showroom_reference_frame_time_publisher.publish(self.spot_info_dict["exercise_data"]["video_frame_idxs"][best_reference_recording_idx][best_reference_frame_index])

        return best_reference_pose, smallest_delta

    @logy.trace_time("calculate_reference_pose_mapping", period=100)
    def calculate_progress(self) -> np.ndarray:
        """Calculate the progress of a user, using a set of reference repetitions.

        This method measures the similarity between the recent feature_trajectory of a user and the vectors
        inside a hankel matrix of the reference trajectory. Thereby we can compute how likely it is that the
        use is at a certain point in the execution of the exercise of the expert.

        Returns:
            reference_pose: The reference pose that we think the user is in
            mean_resampled_values_reference_trajectory_fractions_average_difference: The average difference of "where different features think we are"

        """
        last_progress = self.progress
        predicted_indices = []
        median_resampled_values_reference_trajectory_fractions = []
        progress_vectors = []

        for h, f in self.features.items():
            # For our algorithm, we compare the discretized trajectories of our reference trajectories and our user's trajectory
            discretization_reference_trajectory_indices_tensor = f.discretization_reference_trajectory_indices_tensor
            hankel_tensor_2 = f.hankel_tensor
            discrete_feature_trajectory = np.array(f.discretized_values)

            for recording_idx, hankel_tensor_1 in enumerate(hankel_tensor_2):
                errors = trajectory_distance(hankel_tensor_1, discrete_feature_trajectory, 100, 1)
                prediction = np.argmin(errors)
                index = discretization_reference_trajectory_indices_tensor[recording_idx][prediction]
                predicted_indices.append(index)
            
            median_resampled_values_reference_trajectory_fraction_dict = f.median_trajectory_discretization_ranges[prediction]
            feature_progress = np.mean([median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_from"], median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_to"]])
            progress_vector = map_progress_to_vector(feature_progress)
            progress_vectors.append(progress_vector)
            median_resampled_values_reference_trajectory_fractions.append(median_resampled_values_reference_trajectory_fraction_dict)

        progress_estimate, self.alignment, self.progress_alignment_vector = map_vectors_to_progress_and_alignment(vectors=progress_vectors)

        # We try to calculate a progress velocity here and never use the estimated progress directly but update it based on a dacaying velocityyyyyy
        if self.t - self.last_t:
            delta = 0.5 # We always have to be lower than 50% progress delta
            for rep_term in [0, 1, -1]: # The three repetitions terms let us see what happens if our progression would reach into the next repetition, this repetition or a previous repetition
                delta_candidate = progress_estimate + rep_term - self.progress
                if abs(delta) > abs(delta_candidate):
                    delta = delta_candidate
                    
            self.progress_velocity = self.progress_velocity * self.config['PROGRESS_VELOCITY_EPSILON'] + (delta / (self.t - self.last_t)) * (1 - self.config['PROGRESS_VELOCITY_EPSILON'])
        else:
            self.progress_velocity = 0

        self.progress = (self.progress + (self.t - self.last_t) * self.progress_velocity) % 1
        self.progress_differences_this_rep = np.append(self.progress_differences_this_rep, self.progress - last_progress)

        self.showroom_reference_progress_publisher.publish(int(self.progress * 100))

        return self.progress

    @logy.trace_time("analyze_feature_progressions", period=100)
    def analyze_feature_progressions(self,
        features: dict) -> Tuple[bool, bool, dict, Any]:
        """Detect done and bad repetitions by analyzing the feature's progressions.
        
        Args:
            features: The feature dictionary from the previous analysis step
            
        Returns:
            increase_reps (bool): Is true if a repetition was detected
        """

        total_increase_reps = False
        
        def num_features_to_progress(num_features):
            # Simple formula to determine the number of feature that have to finish before exercise is finished
            return int(num_features/self.config['NUM_FEATURE_TO_PROGRESS_ALPHA'] + self.config['NUM_FEATURE_TO_PROGRESS_BETA'])

        for idx in range(len(next(iter(features.values())).reference_recording_features)): # TODO: The iterator here is a little hacky, come up with something better here
            increase_reps = True
            in_beginning_state = True
            num_features_progressed = 0
            num_features_progressed_too_far = 0
            num_features_in_beginning_state = 0

            for feature in features.values():
                f = feature.reference_recording_features[idx]
                beginning_state = f.median_beginning_state
                number_of_dicided_state_changes_for_repetition = f.number_of_dicided_state_changes
                if f.state == beginning_state:
                    num_features_in_beginning_state += 1
                else:
                    in_beginning_state = False
                if f.progression < number_of_dicided_state_changes_for_repetition:
                    increase_reps = False
                elif f.progression > number_of_dicided_state_changes_for_repetition:
                    num_features_progressed_too_far += 1
                elif f.progression == number_of_dicided_state_changes_for_repetition:
                    num_features_progressed += 1

            # We calculate the number of features that have to have progressed in order to count this repetition
            if num_features_progressed < num_features_to_progress(len(features.values())) and self.config['ENABLE_NUM_FEATURES_TO_PROGRESS_CHECK']: # TODO: This is a parameter to be tuned!!
                increase_reps = False

            # We do not want too many features to progress too far (i.e. to have progressed into the next repetition before we end this repetition)
            if num_features_progressed_too_far * self.config['NUM_FEATURES_PROGRESSED_TOO_FAR_MU'] > num_features_in_beginning_state and self.config['ENABLE_NUM_FEATURES_PROGRESSED_TOO_FAR_CHECK']:
                self.log_with_metadata(logy.debug, "A feature has progressed through too many states. Marking this repetition as bad. Feature specification: " + str(f.specification_dict))
                self.bad_repetition_dict[idx] = True

            # If we are in a beginning state and the repetition is bad, reset and beginn next repetition
            if in_beginning_state and self.bad_repetition_dict.get(idx, None):
                self.log_with_metadata(logy.info, "Bad repetition aborted. Resetting feature progressions and repetition data...")
                for feature in features.values():
                    f = feature.reference_recording_features[idx]
                    f.progression = 0
                self.bad_repetition_dict[idx] = False
                self.beginning_of_next_repetition_detected = True
                self.skelleton_deltas_since_rep_start = []
                
                increase_reps = self.analyze_feature_progressions(features)

            if self.bad_repetition_dict.get(idx, None):
                increase_reps = False

            if increase_reps:
                self.log_with_metadata(logy.debug, "All features have progressed. Repetition detected. Resetting feature progressions...")
                self.beginning_of_next_repetition_detected = True

            # As long as we have not gone into a new repetition (out of the beginning state), we always reset the following things to not clutter measurements over the next repetition
            if not in_beginning_state and self.beginning_of_next_repetition_detected:
                self.log_with_metadata(logy.debug, "Last repetition started and ended, measuring feature alignment and progress differences for new repetition...")
                self.skelleton_deltas_since_rep_start = []
                self.beginning_of_next_repetition_detected = False
                self.alignments_this_rep = np.array([])
                self.progress_differences_this_rep = np.array([])
                self.progress = 0

            # If one reference recording has detected a repetition, we let it count
            if increase_reps:
                rp.logerr(idx)
            total_increase_reps = total_increase_reps or increase_reps

        if total_increase_reps:
            for idx in range(len(next(iter(features.values())).reference_recording_features)):
                for feature in features.values():
                    f = feature.reference_recording_features[idx]
                    f.progression = 0


        return total_increase_reps

    def calculate_repetition_score(self): # TODO: This should be placed in the Algorithm file, but for now needs quite a lot of data from the Worker object so we leave it here
        """ This is a simple first approach to calculating the repetition score for a repetition. 
        
        TODO: Values have been chosen with few shots and should be improved heuristically! 
        """
        x = self.skelleton_deltas_since_rep_start
        alpha = 1/40 # This has to be heuristically determined, as is depends on the size of the normal skelleton
        critical_freq = 0.3 # TODO: Choose heuristically!!

        # Filter scores in order to minimize impact of faulty reference skelleton predictions
        b, a = signal.butter(min(4, max(1, int(len(self.skelleton_deltas_since_rep_start)/4))), critical_freq, analog=False)
        x = signal.filtfilt(b, a, x, padlen=max(0, min(12, len(x) - 1)))

        average_delta = np.median(x) # TODO: Try average and median here, see which one works better
        delta_score = np.absolute(average_delta * alpha)

        delta_score = np.clip(1 - delta_score, 0, 1)

        delta_score = np.power(delta_score, 3)

        # We need our score to be between 0 and 1
        score = 100 * delta_score

        # TODO: Remove these scores. We log them all the time for now to let us choose parameters heuristically
        logy.log_mean("LowestScore:ExerciseName:" + str(self.spot_info_dict['exercise_data']['name']), value=float(np.min(self.skelleton_deltas_since_rep_start)))
        logy.log_mean("HighestScore:ExerciseName:" + str(self.spot_info_dict['exercise_data']['name']), value=float(np.max(self.skelleton_deltas_since_rep_start)))
        logy.log_mean("ExerciseScore:ExerciseName:" + str(self.spot_info_dict['exercise_data']['name']), value=float(score))

        return score