#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Algorithm for our motion analysis.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

import collections
import numpy as np
from typing import Any, Tuple

try:
    from motion_analysis.src.algorithm.AlgoConfig import *
    from motion_analysis.src.algorithm.FeatureExtraction import *
    from motion_analysis.src.algorithm.Features import *
    from motion_analysis.src.algorithm.AlgoUtils import *
    from motion_analysis.src.algorithm.GUI import *
except (ModuleNotFoundError, ImportError):
    from src.algorithm.AlgoConfig import *
    from src.algorithm.FeatureExtraction import *
    from src.algorithm.Features import *
    from src.algorithm.AlgoUtils import *
    from src.algorithm.GUI import *


def trajectory_distance(hankel_matrix: np.ndarray, feature_trajectory: np.ndarray, max_weight: float, min_weight: float):
    """Compute a custom metric that represents the differences between a trajectory and the reference trajectory or shifted versions thereof.

    This function computes the l2 norm of hankel_matrix - feature_trajectory, but influence to the error fades out linearly form the 
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
    hankel_matrix_shortened = hankel_matrix[:, -comparing_length:]
    feature_trajectory_shortened = feature_trajectory[-comparing_length:]
    distances = np.power(hankel_matrix_shortened - feature_trajectory_shortened, 2)
    fading_factor = np.geomspace(min_weight, max_weight, comparing_length) # Let older signals have less influence on the error
    errors = np.linalg.norm(distances * fading_factor, axis=1)
    normed_errors = errors / sum(errors)

    return normed_errors


def analyze_feature_progressions(
    features: dict,
    bad_repetition: bool) -> Tuple[bool, bool, dict, Any]:
    """Detect done and bad repetitions by analyzing the feature's progressions.
    
    Args:
        features: The feature dictionary from the previous comparing step
        bad_repetition: Whether the curernt repetition has already been detected to be bad outside this method
        
    Returns:
        increase_reps (bool): Is true if a repetition was detected
        bad_repetition (bool): Is true if a repetition is deemed bad by this analysis
    """
    increase_reps = True
    in_beginning_state = True

    for f in features.values():
        beginning_state = f.reference_feature_collection.median_beginning_state
        number_of_dicided_state_changes_for_repetition = f.reference_feature_collection.number_of_dicided_state_changes

        if f.state != beginning_state:
            in_beginning_state = False

        if f.progression < number_of_dicided_state_changes_for_repetition:
            increase_reps = False
        elif f.progression > number_of_dicided_state_changes_for_repetition and not MESSY_INPUTS:
            bad_repetition = True

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

    if in_beginning_state and bad_repetition:
        for f in features.values():
            f.progression = 0
        increase_reps, bad_repetition = analyze_feature_progressions(features, False)

    if bad_repetition and not MESSY_INPUTS:
        increase_reps = False

    if increase_reps:
        for f in features.values():
            f.progression = 0

    return increase_reps, bad_repetition


def calculate_reference_pose_mapping(features: dict, exercise_data: dict, gui: MotionAnaysisGUI = None) -> np.ndarray:
    """Calculate the pose in the reference trajectory that we think our user is most probably in.

    This method measures the similarity between the recent feature_trajectory of a user and the vectors
    inside a hankel matrix of the reference trajectory. Thereby we can compute how likely it is that the
    use is at a certain point in the execution of the exercise of the expert.

    Args:
        features: A dictionary that holds a list of past feature values for every type of feature
                                in every feature category
        exercise data: A dictionary containing metadata around the exercise
        gui: A GUI to update with the data of this analysis

    Returns:
        reference_pose: The reference pose that we think the user is in
        mean_resampled_values_reference_trajectory_fractions_average_difference: The average difference of "where different features think we are"

    """
    recordings = exercise_data['recordings']

    if len(recordings.values()) > 1:
        raise NotImplementedError("We have not gotten this method ready for multiple recordings!")

    recording = list(recordings.values())[0]

    predicted_indices = []
    median_resampled_values_reference_trajectory_fractions = []
    progress_vectors = []

    for h, f in features.items():
        # For our algorithm, we compare the discretized trajectories of our reference trajectories and our user's trajectory
        discretization_reference_trajectory_indices_tensor = f.reference_feature_collection.discretization_reference_trajectory_indices_tensor
        hankel_tensor_1 = f.reference_feature_collection.hankel_tensor
        discrete_feature_trajectory = np.array(f.discretized_values)

        for idx, reference_trajectory_hankel_matrix in enumerate(hankel_tensor_1):
            errors = trajectory_distance(reference_trajectory_hankel_matrix, discrete_feature_trajectory, 100, 1)
            prediction = np.argmin(errors)
            index = discretization_reference_trajectory_indices_tensor[idx][prediction]
            median_resampled_values_reference_trajectory_fraction_dict = f.reference_feature_collection.median_trajectory_discretization_ranges[prediction]
            progress = np.mean([median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_from"], median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_to"]])
            progress_vector = map_progress_to_vector(progress)
            progress_vectors.append(progress_vector)
            median_resampled_values_reference_trajectory_fractions.append(median_resampled_values_reference_trajectory_fraction_dict)
            predicted_indices.append(index)

            if gui:
                try:
                    sample_reference_feature = f.reference_feature_collection.reference_recording_features[0]
                    gui.feature_widgets[h].update.emit(np.array(f.values), \
                        np.array(sample_reference_feature.values), \
                            np.array(f.discretized_values), \
                                np.array(sample_reference_feature.discretized_values), \
                                    np.array(errors), \
                                        np.array([0, progress_vector.real, 0, progress_vector.imag]), \
                                            np.array(prediction))
                except Exception as e:
                    pass
                    # rp.logerr(e)
                    # rp.logerr(gui.feature_widgets)
                    # rp.logerr(h)

        # Look at every reference feature separately
        # for r in f.reference_feature_collection.reference_recording_features:
        #     joint_difference = total_joint_difference(pose, reference_pose)
        #     r.moving_average_joint_difference = r.moving_average_joint_difference * JOINT_DIFFERENCE_FADING_FACTOR + r * (1 - JOINT_DIFFERENCE_FADING_FACTOR)


    progress, alignment, progress_alignment_vector = map_vectors_to_progress_and_alignment(vectors=progress_vectors)

    # TODO: update global GUI parameters!!

    median_resampled_values_reference_trajectory_fractions_errors = []
    # TODO: This is a little bit overkill but should still give the correct result, maybe change to something more elegant
    for idx1, value in enumerate(median_resampled_values_reference_trajectory_fractions):
        for idx2 in range(len(median_resampled_values_reference_trajectory_fractions)):
            if idx2 == idx1:
                continue
            median_resampled_values_reference_trajectory_fractions_errors.append(custom_metric(value, median_resampled_values_reference_trajectory_fractions[idx2]))
    
    reference_pose = recording[int(len(recording) * progress)]

    mean_resampled_values_reference_trajectory_fractions_average_difference = np.average(median_resampled_values_reference_trajectory_fractions_errors)/2 # divide by two, since we account for every errors twice
        
    return reference_pose, mean_resampled_values_reference_trajectory_fractions_average_difference


def total_joint_difference(pose: np.ndarray, reference_pose: np.ndarray):
    return np.average(pose - reference_pose)



class Predictor(ABC):
    def __init__(self, features: List[Feature]):
        self.features = features

    @abstractmethod
    def __call__(self, pose: np.ndarray) -> np.ndarray:
        """Updates all features with the pose and predicts the best possible reference pose."""
        raise NotImplementedError


class ExercisePredictor(Predictor):
    """This Predictor lets us predict an exercise out of all exercises."""
    def __init__(self, features: List[Feature]):
        super().__init__(features)
        raise NotImplementedError


class RecordingPredictor(Predictor):
    """This Predictor lets us predict a recording out of several recordings of an exercise."""
    def __init__(self, features: List[Feature]):
        super().__init__(features)
        raise NotImplementedError


class PosePredictor(Predictor):
    """This Predictor lets us predict a pose, given a recording."""
    def __init__(self, features: List[Feature]):
        super().__init__(features)
        
    # def __call__(self, pose: np.ndarray):
        # # TODO: This should be only one dimension in the hanel tensor. Check if this works!!!
        # reference_trajectory_hankel_matrix = self.hankel_tensor[0]
        # errors = trajectory_distance(reference_trajectory_hankel_matrix, feature.discrete_feature_trajectory, 100, 1)
        # prediction = np.argmin(errors)
        # # TODO: This should be only one dimension in this tensor. Check if this works!!!
        # self.index = self.discretization_reference_trajectory_indices_tensor[0][prediction]
        # median_resampled_values_reference_trajectory_fraction_dict = self.median_trajectory_discretization_ranges[prediction]
        # progress = np.mean([median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_from"], median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_to"]])
        # self.progress_vector = map_progress_to_vector(progress)
        # self.median_resampled_values_reference_trajectory_fraction_dict = median_resampled_values_reference_trajectory_fraction_dict
        # self.reference_pose = self.recording[int(len(self.recording) * progress)]

        # joint_difference = total_joint_difference(pose, self.reference_pose)
        # self.moving_average_total_joint_difference = self.moving_average_total_joint_difference * JOINT_DIFFERENCE_FADING_FACTOR + joint_difference * (1 - JOINT_DIFFERENCE_FADING_FACTOR)
        # self.total_joint_differences_this_rep.append(joint_difference)

