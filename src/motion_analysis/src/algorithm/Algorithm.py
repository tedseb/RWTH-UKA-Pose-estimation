#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Algorithm that for our motion analysis.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

import collections
import numpy as np

try:
    from motion_analysis.src.algorithm.AlgoConfig import *
    from motion_analysis.src.algorithm.FeatureExtraction import *
    from motion_analysis.src.algorithm.AlgoUtils import *
except (ModuleNotFoundError, ImportError):
    from src.algorithm.AlgoConfig import *
    from src.algorithm.FeatureExtraction import *
    from src.algorithm.AlgoUtils import *


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
    # rp.logerr("hankel_matrix_shortened:" + str(hankel_matrix_shortened))
    feature_trajectory_shortened = feature_trajectory[-comparing_length:]
    # rp.logerr("feature_trajectory_shortened:" + str(feature_trajectory_shortened))
    distances = np.power(hankel_matrix_shortened - feature_trajectory_shortened, 2)
    # rp.logerr("distances:" + str(distances))
    fading_factor = np.geomspace(min_weight, max_weight, comparing_length) # Let older signals have less influence on the error
    errors = np.linalg.norm(distances * fading_factor, axis=1)
    # rp.logerr("fading_factor:" + str(fading_factor))
    normed_errors = errors / sum(errors)
    # rp.logerr("normed_errors:" + str(normed_errors))
    return normed_errors


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
            scale = exercise_data['reference_feature_data'][feature_type][k]['scale']
            try:
                last_resampled_feature_value = float(last_resampled_features[feature_type][k][-1])
                new_resampled_feature_values = discritize_feature_values(feature_value, last_resampled_feature_value, resolution)
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
                rp.logerr("new feature progression:" + str(new_feature_progression))
                if not MESSY_INPUTS:
                    bad_repetition = True

                    if bad_repetition == False:
                        rp.logerr("bad_repetition detected : Too much feature progression")

        # If a data type has no updates, remove it again
        if new_feature_progressions[feature_type] == {}:
            del new_feature_progressions[feature_type]
        if new_resampled_features[feature_type] == {}:
            del new_resampled_features[feature_type]

    if in_beginning_state and bad_repetition:
        new_feature_progressions = reset_child_featuers(new_feature_progressions)
        increase_reps, bad_repetition, new_feature_progressions, new_resampled_features = compare_high_level_features(spot_info_dict, new_feature_progressions, last_resampled_features, features_states, False)

    if bad_repetition:
        increase_reps = False

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

    reference_poses = exercise_data['recording']

    predicted_indices = []
    median_resampled_values_reference_trajectory_fractions = []
    progress_vectors = []

    for feature_type, features in exercise_data['reference_feature_data'].items():
        for k, v in features.items():
            discretization_reference_trajectory_indices_tensor = v['discretization_reference_trajectory_indices_tensor']
            hankel_tensor = v['hankel_tensor']
            feature_trajectory = np.asarray(feature_trajectories[feature_type][k], np.float16)

            for idx, reference_trajectory_hankel_matrix in enumerate(hankel_tensor):
                errors = custom_metric(reference_trajectory_hankel_matrix, feature_trajectory, 100, 1)
                prediction = np.argmin(errors)
                index = discretization_reference_trajectory_indices_tensor[idx][prediction]
                median_resampled_values_reference_trajectory_fraction_dict = v['median_resampled_values_reference_trajectory_fractions'][prediction]
                progress = np.mean([median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_from"], median_resampled_values_reference_trajectory_fraction_dict["median_resampled_values_reference_trajectory_fraction_to"]])
                # rp.logerr(v['median_resampled_values_reference_trajectory_fractions'])
                # rp.logerr(median_resampled_values_reference_trajectory_fraction_dict)
                # rp.logerr(progress)
                progress_vectors.append(map_progress_to_vector(progress))
                median_resampled_values_reference_trajectory_fractions.append(median_resampled_values_reference_trajectory_fraction_dict)

                predicted_indices.append(index)

    progress, alignment, progress_alignment_vector = map_vectors_to_progress_and_alignment(vectors=progress_vectors)

    # rp.logerr(progress_vectors)
    # rp.logerr("progress:" + str(progress))
    # rp.logerr("alignment:" + str(alignment))

    median_resampled_values_reference_trajectory_fractions_errors = []
    # TODO: This is a little bit overkill but should still give the correct result, maybe change to something more elegant
    for idx1, value in enumerate(median_resampled_values_reference_trajectory_fractions):
        for idx2 in range(len(median_resampled_values_reference_trajectory_fractions)):
            if idx2 == idx1:
                continue
            median_resampled_values_reference_trajectory_fractions_errors.append(my_weird_metric(value, median_resampled_values_reference_trajectory_fractions[idx2]))
    
    # reference_pose_index = int(np.mean((predicted_indices)))
    reference_pose = reference_poses[int(len(reference_poses) * progress)]

    # rp.logerr(predicted_indices)
    # rp.logerr(reference_pose_index)
    # rp.logerr(discretization_reference_trajectory_indices_tensor)
    # rp.logerr(len(reference_poses))

    mean_resampled_values_reference_trajectory_fractions_average_difference = np.average(median_resampled_values_reference_trajectory_fractions_errors)/2 # divide by two, since we account for every errors twice
        
    return reference_pose, mean_resampled_values_reference_trajectory_fractions_average_difference

