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
    from motion_analysis.src.algorithm.FeatureExtraction import *
    from motion_analysis.src.algorithm.Features import *
    from motion_analysis.src.algorithm.AlgoUtils import *
    from motion_analysis.src.algorithm.GUI import *
except (ModuleNotFoundError, ImportError):
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

