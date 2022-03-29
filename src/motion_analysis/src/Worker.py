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
from tkinter import E
from traceback import format_exc
from typing import NoReturn, Dict
from collections import defaultdict

import numpy as np
import rospy as rp
from std_msgs.msg import String, Int16
from unittest import mock
from scipy import signal
import logy

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
        logy.basic_config(debug_level=logy.DEBUG, module_name="Worker")
        self.config = config

        self.spot_queue_interface = spot_queue_interface_class(self.config)
        self.spot_metadata_interface = spot_metadata_interface_class()
        self.pose_definition_adapter = pose_definition_adapter_class()
        # Use the same interface class for these two, because they have the same needs to the interface
        self.features_interface = features_interface_interface_class()

        self.running = True

        self.user_exercise_state_publisher = rp.Publisher(
            self.config['ROS_TOPIC_USER_EXERCISE_STATES'], String, queue_size=1000)
        self.user_correction_publisher = rp.Publisher(
            self.config['ROS_TOPIC_USER_CORRECTIONS'], String, queue_size=1000)

        self.showroom_reference_progress_publisher = rp.Publisher(
            self.config['SHOW_ROOM_REFERENCE_PROGRESS_TOPIC'], Int16, queue_size=1000)
        self.showroom_video_reference_publisher = rp.Publisher(
            self.config['SHOW_ROOM_VIDEO_REFERENCE'], String, queue_size=1000)

        self.predicted_skelleton_publisher = rp.Publisher(
            "motion_analysis_reference_prediction", Person, queue_size=1000)
        self.user_skelleton_publisher = rp.Publisher(
            "motion_analysis_input", Person, queue_size=1000)

        self.spot_key = spot_key

        self.features = {}

        self.bad_repetition_dict = {}

        self.gui = gui

        self.last_score = 0

        self.t = None
        self.progress = 0
        self.progress_velocity = 0
        self.progress_alignment_vector = None

        self.spot_info_dict = None

        self.skelleton_deltas_since_rep_start = []

        self.last_worst_recordings_mapping_delta_ms = 0
        self.best_reference_recording_idx = 0
        
        self._t = 0
        self.t_0 = 0

        self.start()

    def log_with_metadata(self, logger: object, msg: str) -> NoReturn:
        """ Helper function to log stuff under reference of the spot key and exercise name """
        logger("Spot:" + str(self.spot_key) + ": ExerciseName: " +
               str(self.spot_info_dict['exercise_data']['name']) + ": " + str(msg))

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
        msg.bodyParts = self.pose_definition_adapter.ndarray_to_body_parts(
            pose)
        publisher.publish(msg)

    def logtime(self, _id): # TODO: Remove me!
        """A utility function to time this code and find bottlenecks."""
        t_old = self._t
        self._t = (time.time_ns() / 1e+6) - self.t_0
        t_d =  self._t - t_old
        rp.logerr(str(_id) + ": t:" + str(self._t) + " td: " + str(t_d))

    def run(self) -> NoReturn:
        _, _, spot_info_key, spot_feature_key = generate_redis_key_names(
            self.spot_key, self.config)
        # Fetch last feature data
        self.features = self.features_interface.get(spot_feature_key)
        for f in self.features.values():
            for rf in f.reference_recording_features:
                rf.reset_live_trajectories()

        # We start with all repetitions being "bad" such that we have to enter a valid beginning state before we start counting
        # TODO: The iterator here is a little hacky and only works if all recordings use the same features, come up with something better here
        for idx in range(len(next(iter(self.features.values())).reference_recording_features)):
            self.bad_repetition_dict[idx] = True

        def array_factory():
            return np.array([0])
        
        self.bad_alignments_this_rep = 0
        self.alignments_this_rep = np.array([0.7])
        self.progress_differences_this_rep = defaultdict(array_factory)

        def false_factory():
            return False
        self.beginning_of_next_repetition_detected = defaultdict(false_factory)

        # The following lines fetch data that we need to analyse
        self.spot_info_dict = self.spot_metadata_interface.get_spot_info_dict(
            spot_info_key, ["exercise_data_hash", "start_time", "repetitions", "station_usage_hash"])

        while(self.running):
            try:
                with logy.TraceTime("motion_analysis_full_algorithm_iter_t"):
                    self.t_0 = time.time_ns() / 1e+6
                    self._t = 0

                    self.spot_info_dict.update(self.spot_metadata_interface.get_spot_info_dict(
                        spot_info_key, ["exercise_data_hash", "start_time", "station_usage_hash"]))
                    self.spot_info_dict.update(self.get_exercise_data(
                        spot_info_key, self.spot_info_dict["exercise_data_hash"]))

                    # As long as there are skelletons available for this spot, continue
                    past_joints_with_timestamp_list, present_joints_with_timestamp, future_joints_with_timestamp_list = self.spot_queue_interface.dequeue(
                        self.spot_key)

                    # Extract feature states
                    pose = present_joints_with_timestamp['used_joint_ndarray']
                    self.last_t = self.t if self.t else present_joints_with_timestamp[
                        'ros_timestamp']
                    self.t = present_joints_with_timestamp['ros_timestamp']
                    pose = self.pose_definition_adapter.normalize_skelleton(
                        pose)

                    for f in self.features.values():
                        f.update_pose(pose, self.pose_definition_adapter)
                        for rf in f.reference_recording_features:
                            rf.update_pose(pose, self.pose_definition_adapter)

                    # Calculate poses before we test for repetition increase, because that might erase feature values for this repetition!
                    # Calculate a new reference pose mapping
                    reference_pose, delta = self.compute_reference_pose_mapping_data(
                        pose)

                    self.skelleton_deltas_since_rep_start.append(delta)
                    self.skelleton_deltas_since_rep_start = self.skelleton_deltas_since_rep_start[-60:] # TODO: Parameterize this! 
                    score = self.calculate_repetition_score()


                    # Compare joints with expert system data
                    increase_reps, increase_recording_idx = self.detect_repetition()

                    # See if progress jumped around too much
                    if increase_reps:
                        negative_progress_differences = self.progress_differences_this_rep[increase_recording_idx][np.where(
                            self.progress_differences_this_rep[increase_recording_idx] < 0)]
                        positive_progresses_differences = self.progress_differences_this_rep[increase_recording_idx][np.where(
                            self.progress_differences_this_rep[increase_recording_idx] > 0)]
                        if len(negative_progress_differences) > 0:
                            neg_mean = abs(
                                np.mean(negative_progress_differences))
                        else:
                            neg_mean = 0
                        if len(positive_progresses_differences) > 0:
                            pos_median = abs(
                                np.median(positive_progresses_differences))
                        else:
                            pos_median = 0
                        # We do not want negative and positive progresses to relate to each other as follows: (If they do, we mark this repetition as bad)
                        if self.config['ENABLE_JUMPY_PROGRESS_CHECK'] and (neg_mean > pos_median * self.config['JUMPY_PROGRESS_ALPHA']) and self.config['JUMPY_PROGRESS_BETA'] * len(negative_progress_differences) > len(positive_progresses_differences):
                            self.log_with_metadata(logy.debug, "Progression was too jumpy for recording in this repetition. Marking this repetition as bad...")
                            for k in self.bad_repetition_dict.keys():
                                self.bad_repetition_dict[k] = True
                            increase_reps = False
                            
                        # If the feature alignments in this repetitions are too bad, we do not count the repetition for this recording
                        if score < self.config['MINMUM_ALLOWED_SCORE'] and self.config['ENABLE_SCORE_CHECK']:
                            self.log_with_metadata(logy.debug, "Bad score" + str(score) + " for recording " + str(
                                increase_recording_idx) + "  during this repetition. Repetition falsified.")
                            self.bad_repetition_dict[increase_recording_idx] = True
                            increase_reps = False

                        # If the feature alignments in this repetitions are too bad, we do not count the repetition for this recording
                        if increase_reps and \
                            (self.bad_alignments_this_rep >= 30) and \
                            (len(self.alignments_this_rep)/self.bad_alignments_this_rep < 6)  and \
                                self.config['ENABLE_FEATURE_ALIGNMENT_CHECK']:
                            self.log_with_metadata(logy.debug, "Feature missalignment for recording " + str(
                                increase_recording_idx) + "  during this repetition. Repetition falsified.")
                            self.bad_repetition_dict[increase_recording_idx] = True
                            increase_reps = False


                    update_gui_progress(self.gui, self.progress, np.mean(self.alignments_this_rep), self.progress_alignment_vector, score, self.last_score)

                    # Send info back to App
                    if increase_reps:
                        # If repetition is actually increased, set all features to bad rep
                        for idx in range(len(next(iter(self.features.values())).reference_recording_features)):
                            for feature in self.features.values():
                                f = feature.reference_recording_features[idx]
                                f.progression = 0
                                if idx != increase_recording_idx:
                                    self.bad_repetition_dict[idx] = True

                        self.last_score = score
                        self.spot_info_dict['repetitions'] = int(
                            self.spot_info_dict['repetitions']) + 1
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
                        publish_message(self.user_exercise_state_publisher,
                                        self.config['ROS_TOPIC_USER_EXERCISE_STATES'], user_state_data)

                    # Publish poses on ROS to observe in RVIZ
                    self.publish_pose(
                        reference_pose, self.predicted_skelleton_publisher)
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
                        publish_message(
                            self.user_correction_publisher, self.config['ROS_TOPIC_USER_CORRECTIONS'], user_correction_message)

            except QueueEmpty:
                continue
            except Exception as e:
                logy.error_throttle(
                    "Encountered an error during motion analysis :" + format_exc(), throttel_time_ms=3000)

        # # Enqueue data for feature progressions and resampled feature lists
        # self.features_interface.set(self.spot_key, self.features)
        # self.spot_metadata_interface.set_spot_info_dict(
        #     spot_info_key, self.spot_info_dict)

        self.spot_info_dict = None
        self.skelleton_deltas_since_rep_start = []
        self.features = {}
        self.last_score = 0

        self.t = None
        self.progress = 0
        self.progress_velocity = 0
        self.progress_alignment_vector = None
        self.bad_repetition_dict = {}

    @logy.trace_time("compute_reference_pose_mapping_data", period=100)
    def compute_reference_pose_mapping_data(self, pose) -> np.ndarray:
        """Calculate the pose in the reference trajectory that we think our user is most probably in.

        This method measures the similarity between the recent feature_trajectory of a user and the vectors
        inside separate hankel matrices per recording. Apart from the reference pose, progress and other data is computed.

        Returns:
            pose: The reference pose that we think the user is in
            smallest_delta: The smallest delta we could find, i.e. how good is the pose of our user

        """
        recordings = self.spot_info_dict['exercise_data']['recordings']

        predicted_indices = [[] for h in recordings]
        median_resampled_values_reference_trajectory_fractions = [
            [] for h in recordings]
        progress_vectors = [[] for h in recordings]

        compute_all = False
        if self.t - self.last_worst_recordings_mapping_delta_ms >= self.config["MAP_WORST_RECORDINGS_EVERY_N_S"]:
            # If MAP_WORST_RECORDINGS_EVERY_N_MS ms have passed, compute all mappings
            self.last_worst_recordings_mapping_delta_ms = self.t
            compute_all = True

        for h, f in self.features.items():  # Iterate over features
            # Iterate over reference features
            for recording_idx, rf in enumerate(f.reference_recording_features):
                if recording_idx == self.best_reference_recording_idx or recordings[recording_idx].get("video_file_name", False):
                    # If this is the best recording or a video recording, compute at every step
                    pass
                elif not compute_all:
                    continue

                # For our algorithm, we compare the discretized trajectories of our reference trajectories and our user's trajectory
                discretization_reference_trajectory_indices_tensor = rf.discretization_reference_trajectory_indices_tensor[
                    0]
                discrete_feature_trajectory = np.array(f.discretized_values)
                errors = trajectory_distance(
                    rf.hankel_tensor[0], discrete_feature_trajectory, 100, 1)
                prediction = np.argmin(errors)
                index = discretization_reference_trajectory_indices_tensor[prediction]
                median_resampled_values_reference_trajectory_fraction_dict = rf.median_trajectory_discretization_ranges[
                    prediction]

                feature_progress = np.mean([median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_from"],
                                           median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_to"]])
                progress_vector = map_progress_to_vector(feature_progress)
                progress_vectors[recording_idx].append(progress_vector)
                median_resampled_values_reference_trajectory_fractions[recording_idx].append(
                    median_resampled_values_reference_trajectory_fraction_dict)
                predicted_indices[recording_idx].append(index)

            # For now, these are only for visualization and se we simply set the errors in the feature to the errors for the last recording. Our GUI does not permit anything else
            f.errors = errors
            f.progress_vector = progress_vector
            f.prediction = prediction
            
            update_gui_features(self.gui, f)

        smallest_delta = np.inf
        best_reference_pose = None
        best_reference_frame_index = 0

        progress_estimates = []
        progress_alignment_vectors = []
        best_alignment = 0
        # For a set of progress vectors (one set per recording), we look for the one that leads us to a minimal delta, i.e. the best expert pose
        for idx, vectors in enumerate(progress_vectors):
            recording = recordings[idx]['recording']
            progress_estimate, alignment, progress_alignment_vector = map_vectors_to_progress_and_alignment(
                vectors=vectors)
            
            reference_frame_index = int(len(recording) * progress_estimate)
            reference_pose = recording[reference_frame_index]

            if recordings[idx].get("video_file_name", False):
                showroom_video_reference_message = {
                    'video_file_name': recordings[idx]["video_file_name"],
                    'video_frame_idx': recordings[idx]["video_frame_idcs"][reference_frame_index],
                }
                publish_message(self.showroom_video_reference_publisher,
                                self.config['SHOW_ROOM_VIDEO_REFERENCE'], showroom_video_reference_message)

            else:  # Otherwise, we would like to compute a delta and see if this the reference pose matches the user pose etc.
                # We calculate the differences between elements of our skelleton
                # TODO: Next step: Calculate difference between elements that we put emphasis on in expert system! (Like certain angles or joints!)
                delta = self.pose_definition_adapter.pose_delta(
                    pose, reference_pose)
                progress_estimates.append(progress_estimate)
                progress_alignment_vectors.append(progress_alignment_vector)

                if delta < smallest_delta:
                    smallest_delta = delta
                    best_alignment = alignment
                    self.best_reference_pose = reference_pose
                    self.best_reference_frame_index = reference_frame_index
                    self.best_reference_recording_idx = idx

        self.progress_alignment_vector = np.mean(progress_alignment_vectors)
        progress = self.progress
        progress_velocity = self.progress_velocity
        last_progress = progress

        if best_alignment:
            self.alignments_this_rep = np.append(self.alignments_this_rep, best_alignment)
            if best_alignment < self.config["MINIMAL_ALLOWED_MEAN_FEATURE_ALIGNMENT"]:
                self.bad_alignments_this_rep += 1

        # We try to calculate a progress velocity here and never use the estimated progress directly but update it based on a dacaying velocityyyyyy
        if self.t - self.last_t:
            delta = 0.5  # We always have to be lower than 50% progress delta
            for rep_term in [0, 1, -1]:  # The three repetitions terms let us see what happens if our progression would reach into the next repetition, this repetition or a previous repetition
                delta_candidate = progress_estimate + rep_term - progress
                if abs(delta) > abs(delta_candidate):
                    delta = delta_candidate

            progress_velocity = progress_velocity * self.config['PROGRESS_VELOCITY_EPSILON'] + (
                delta / (self.t - self.last_t)) * (1 - self.config['PROGRESS_VELOCITY_EPSILON'])
        else:
            progress_velocity = 0

        progress = (progress + (self.t - self.last_t)
                         * progress_velocity) % 1
        self.progress_differences_this_rep[idx] = np.append(
            self.progress_differences_this_rep[idx], self.progress - last_progress)

        self.showroom_reference_progress_publisher.publish(
            int(self.progress * 100))

        if compute_all:
            self.progress = np.mean(progress_estimates)
            self.progress_velocity = progress_velocity

        return self.best_reference_pose, smallest_delta

    @logy.trace_time("detect_repetition", period=100)
    def detect_repetition(self) -> Tuple[bool, bool, dict, Any]:
        """Detect "dones" and bad repetitions by analyzing the feature's progressions.

        This method assumes that we have already added the current skelleton to all features and calculated filtered values and such.
        Features calculate if their boundaries have been crossed and we check this here for all features and all recordings
        to dicide wether we count a repetition or not.

        Returns:
            increase_reps (bool): Is true if a repetition was detected
            rep_recording_idx (int): If increase_reps is true, rep_recording_idx is the index of the repeition recording that increased the reps
        """
        total_increase_reps = False
        rep_recording_idx = None
        recordings = self.spot_info_dict['exercise_data']['recordings']

        def num_features_to_progress(num_features):
            # Simple formula to determine the number of feature that have to finish before exercise is finished
            return int((num_features/self.config['NUM_FEATURE_TO_PROGRESS_ALPHA']) + self.config['NUM_FEATURE_TO_PROGRESS_BETA'])

        # TODO: The iterator here is a little hacky and only works if all recordings use the same features, come up with something better here
        for idx in range(len(next(iter(self.features.values())).reference_recording_features)):
            if not recordings[idx]["is_reference_recording"]:
                continue
            increase_reps = True
            in_beginning_state = True
            num_features_progressed = 0
            num_features_progressed_too_far = 0
            num_features_in_beginning_state = 0

            for feature in self.features.values():
                f = feature.reference_recording_features[idx]
                beginning_state = f.median_beginning_state
                number_of_dicided_state_changes_for_repetition = f.number_of_dicided_state_changes

                if f.state == beginning_state:
                    num_features_in_beginning_state += 1
                else:
                    in_beginning_state = False
                    increase_reps = False
                if f.progression < number_of_dicided_state_changes_for_repetition:
                    increase_reps = False
                elif f.progression > number_of_dicided_state_changes_for_repetition:
                    num_features_progressed_too_far += 1
                elif f.progression == number_of_dicided_state_changes_for_repetition:
                    num_features_progressed += 1

            # We calculate the number of features that have to have progressed in order to count this repetition
            # TODO: This is a parameter to be tuned!!
            if increase_reps and (num_features_progressed < num_features_to_progress(len(self.features.values())) and self.config['ENABLE_NUM_FEATURES_TO_PROGRESS_CHECK']):
                self.log_with_metadata(logy.debug, "Too many featuers have progressed!")
                self.bad_repetition_dict[idx] = True
            elif not in_beginning_state and not self.config['ENABLE_NUM_FEATURES_TO_PROGRESS_CHECK']:
                self.bad_repetition_dict[idx] = True

            # We do not want too many features to progress too far (i.e. to have progressed into the next repetition before we end this repetition)
            if num_features_progressed_too_far * self.config['NUM_FEATURES_PROGRESSED_TOO_FAR_MU'] > num_features_in_beginning_state and self.config['ENABLE_NUM_FEATURES_PROGRESSED_TOO_FAR_CHECK']:
                self.log_with_metadata(
                    logy.debug, "A feature has progressed through too many states. Marking this repetition as bad. Feature specification: " + str(f.specification_dict))
                self.bad_repetition_dict[idx] = True

            # Before we check if if we want to reset anything, increase reps needs to be set
            if self.bad_repetition_dict.get(idx, False):
                increase_reps = False

            # If we are in a beginning state and the repetition is bad, reset and beginn next repetition
            if in_beginning_state and self.bad_repetition_dict.get(idx, False):
                self.log_with_metadata(
                    logy.debug, "Bad repetition aborted. Resetting feature progressions and repetition data at recording " + str(idx) + "...")
                for feature in self.features.values():
                    f = feature.reference_recording_features[idx]
                    f.progression = 0
                self.bad_repetition_dict[idx] = False
                self.beginning_of_next_repetition_detected[idx] = True

            # If we detect a repetition, reset and beginn next repetition
            if increase_reps:
                self.log_with_metadata(
                    logy.debug, "All features have progressed. Repetition detected. Resetting feature progressions at recording " + str(idx) + "...")
                for feature in self.features.values():
                    f = feature.reference_recording_features[idx]
                    f.progression = 0
                self.bad_repetition_dict[idx] = False
                self.beginning_of_next_repetition_detected[idx] = True

            # As long as we have not gone into a new repetition (out of the beginning state), we always reset the following things to not clutter measurements over the next repetition
            if not in_beginning_state and self.beginning_of_next_repetition_detected[idx]:
                self.log_with_metadata(
                    logy.debug, "Last repetition ended, new repetition started. Statistic reset at recording " + str(idx) + "...")
                self.skelleton_deltas_since_rep_start = self.skelleton_deltas_since_rep_start[-5:] # TODO: Possibly introduce new HP that controlls the '5' in this term
                self.beginning_of_next_repetition_detected[idx] = False
                self.bad_alignments_this_rep = 0
                self.alignments_this_rep = self.alignments_this_rep[-5:]
                self.progress_differences_this_rep[idx]= np.array([0])
                self.progress = 0

            # If one reference recording has detected a repetition, we let it count
            if increase_reps and not total_increase_reps:
                rep_recording_idx = idx
            total_increase_reps = total_increase_reps or increase_reps

        return total_increase_reps, rep_recording_idx

    # TODO: This should be placed in the Algorithm file, but for now needs quite a lot of data from the Worker object so we leave it here
    def calculate_repetition_score(self):
        """ Calculates the current repetition score.

        This method implements a nonlinear function that maps a set of skelleton deltas to a score between 0 and 100.
        The parameters for this can be configured and have been heuristically chosen by me, Artur.

        Returns:
            score (int): The score, a number between 0 and 100
        """
        x = self.skelleton_deltas_since_rep_start
        x = np.absolute(x) * self.config["SCORE_ALPHA"]
        x = np.median(x)
        
        x = np.clip(1 - x + self.config["SCORE_BETA"], 0, 1)
        x = (np.tanh((x - 0.5) * self.config["SCORE_SIGMA"]) / 2 ) + 0.5
        x = np.clip(np.power(x, self.config["SCORE_GAMMA"]), 0, 1)
        x = np.clip(x + self.config["SCORE_YOTTA"], 0, 1)

        # We need our score to be between 0 and 100
        score = 100 * x

        # TODO: Remove these scores. We log them all the time for now to let us choose parameters heuristically
        logy.log_mean("LowestScore:ExerciseName:" + str(self.spot_info_dict['exercise_data']['name']), value=float(
            np.min(self.skelleton_deltas_since_rep_start)))
        logy.log_mean("HighestScore:ExerciseName:" + str(
            self.spot_info_dict['exercise_data']['name']), value=float(np.max(self.skelleton_deltas_since_rep_start)))
        logy.log_mean("ExerciseScore:ExerciseName:" +
                      str(self.spot_info_dict['exercise_data']['name']), value=float(score))

        return score
