#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Algorithm that for our motion analysis.
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


def custom_metric(hankel_matrix, feature_trajectory, max_weight, min_weight):
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
    feature_trajectory = remove_jitter_from_trajectory(feature_trajectory, REMOVE_JITTER_RANGE)
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
        # With the current feature value and the last resampled feature value, we can compute new resampled feature values
        beginning_state = f.reference_feature_collection.median_beginning_state
        number_of_dicided_state_changes_for_repetition = f.reference_feature_collection.number_of_dicided_state_changes

        if f.state != beginning_state:
            in_beginning_state = False

        if f.progression < number_of_dicided_state_changes_for_repetition:
            increase_reps = False
        elif f.progression > number_of_dicided_state_changes_for_repetition:
            bad_repetition = True

    if in_beginning_state and bad_repetition:
        for f in features.values():
            f.progression = 0
        increase_reps, bad_repetition, features = analyze_feature_progressions(features, False)

    if bad_repetition:
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

    Returns:
        reference_pose: The reference pose that we think the user is in
    """
    def my_weird_metric(a, b):
        """Calculate the absolute difference between two ranges on a "ring" scale between 0 and 1."""
        a_from = a["median_resampled_values_reference_trajectory_fraction_from"]
        a_to = a["median_resampled_values_reference_trajectory_fraction_to"]
        b_from = b["median_resampled_values_reference_trajectory_fraction_from"]
        b_to = b["median_resampled_values_reference_trajectory_fraction_to"]

        if b_from <= a_from <= b_to or b_from <= a_to <= b_to:
            return 0
        else:
            return min([abs(a_from - b_to), abs(b_from - a_to), abs(a_from + 1 - b_to), abs(b_from + 1 - a_to)])

    recordings = exercise_data['recordings']

    if len(recordings.values()) > 1:
        raise NotImplementedError("We have not gotten this method ready for multiple recordings!")

    recording = list(recordings.values())[0]

    predicted_indices = []
    median_resampled_values_reference_trajectory_fractions = []
    progress_vectors = []

    for h, f in features.items():
        discretization_reference_trajectory_indices_tensor = f.reference_feature_collection.discretization_reference_trajectory_indices_tensor
        hankel_tensor = f.reference_feature_collection.hankel_tensor
        discrete_feature_trajectory = f.discretized_values

        for idx, reference_trajectory_hankel_matrix in enumerate(hankel_tensor):
            errors = custom_metric(reference_trajectory_hankel_matrix, discrete_feature_trajectory, 100, 1)
            prediction = np.argmin(errors)
            index = discretization_reference_trajectory_indices_tensor[idx][prediction]
            median_resampled_values_reference_trajectory_fraction_dict = f.reference_feature_collection.median_trajectory_discretization_ranges[prediction]
            progress = np.mean([median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_from"], median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_to"]])
            progress_vector = map_progress_to_vector(progress)
            progress_vectors.append(progress_vector)
            median_resampled_values_reference_trajectory_fractions.append(median_resampled_values_reference_trajectory_fraction_dict)
            predicted_indices.append(index)

            if gui:
                sample_reference_feature = f.reference_feature_collection.reference_features[0]
                gui.feature_widgets[h].update(f.values, \
                    sample_reference_feature.values, \
                        f.discretized_values, \
                            sample_reference_feature.discretized_values, \
                                errors, \
                                    progress_vector, \
                                        index)

    progress, alignment, progress_alignment_vector = map_vectors_to_progress_and_alignment(vectors=progress_vectors)

    if gui:
        gui.

    median_resampled_values_reference_trajectory_fractions_errors = []
    # TODO: This is a little bit overkill but should still give the correct result, maybe change to something more elegant
    for idx1, value in enumerate(median_resampled_values_reference_trajectory_fractions):
        for idx2 in range(len(median_resampled_values_reference_trajectory_fractions)):
            if idx2 == idx1:
                continue
            median_resampled_values_reference_trajectory_fractions_errors.append(my_weird_metric(value, median_resampled_values_reference_trajectory_fractions[idx2]))
    
    reference_pose = recording[int(len(recording) * progress)]

    mean_resampled_values_reference_trajectory_fractions_average_difference = np.average(median_resampled_values_reference_trajectory_fractions_errors)/2 # divide by two, since we account for every errors twice
        
    return reference_pose, mean_resampled_values_reference_trajectory_fractions_average_difference

