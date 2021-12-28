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

# From https://simple-pid.readthedocs.io/en/latest/_modules/simple_pid/PID.html

def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value
    
class PID(object):
    """A simple PID controller."""

    def __init__(
        self,
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        setpoint=0,
        sample_time=0.01,
        output_limits=(None, None),
        auto_mode=True,
        proportional_on_measurement=False,
        error_map=None,
    ):
        """
        Initialize a new PID controller.

        :param Kp: The value for the proportional gain Kp
        :param Ki: The value for the integral gain Ki
        :param Kd: The value for the derivative gain Kd
        :param setpoint: The initial setpoint that the PID will try to achieve
        :param sample_time: The time in seconds which the controller should wait before generating
            a new output value. The PID works best when it is constantly called (eg. during a
            loop), but with a sample time set so that the time difference between each update is
            (close to) constant. If set to None, the PID will compute a new output value every time
            it is called.
        :param output_limits: The initial output limits to use, given as an iterable with 2
            elements, for example: (lower, upper). The output will never go below the lower limit
            or above the upper limit. Either of the limits can also be set to None to have no limit
            in that direction. Setting output limits also avoids integral windup, since the
            integral term will never be allowed to grow outside of the limits.
        :param auto_mode: Whether the controller should be enabled (auto mode) or not (manual mode)
        :param proportional_on_measurement: Whether the proportional term should be calculated on
            the input directly rather than on the error (which is the traditional way). Using
            proportional-on-measurement avoids overshoot for some types of systems.
        :param error_map: Function to transform the error value in another constrained value.
        """
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        self._min_output, self._max_output = None, None
        self._auto_mode = auto_mode
        self.proportional_on_measurement = proportional_on_measurement
        self.error_map = error_map

        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_time = None
        self._last_output = None
        self._last_input = None

        self.output_limits = output_limits
        self.reset()


    def __call__(self, input_, dt=None):
        """
        Update the PID controller.

        Call the PID controller with *input_* and calculate and return a control output if
        sample_time seconds has passed since the last update. If no new output is calculated,
        return the previous output instead (or None if no value has been calculated yet).

        :param dt: If set, uses this value for timestep instead of real time. This can be used in
            simulations when simulation time is different from real time.
        """
        if not self.auto_mode:
            return self._last_output

        now = _current_time()
        if dt is None:
            dt = now - self._last_time if (now - self._last_time) else 1e-16
        elif dt <= 0:
            raise ValueError('dt has negative value {}, must be positive'.format(dt))

        if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
            # Only update every sample_time seconds
            return self._last_output

        # Compute error terms
        error = self.setpoint - input_
        d_input = input_ - (self._last_input if (self._last_input is not None) else input_)

        # Check if must map the error
        if self.error_map is not None:
            error = self.error_map(error)

        # Compute the proportional term
        if not self.proportional_on_measurement:
            # Regular proportional-on-error, simply set the proportional term
            self._proportional = self.Kp * error
        else:
            # Add the proportional error on measurement to error_sum
            self._proportional -= self.Kp * d_input

        # Compute integral and derivative terms
        self._integral += self.Ki * error * dt
        self._integral = _clamp(self._integral, self.output_limits)  # Avoid integral windup

        self._derivative = -self.Kd * d_input / dt

        # Compute final output
        output = self._proportional + self._integral + self._derivative
        output = _clamp(output, self.output_limits)

        # Keep track of state
        self._last_output = output
        self._last_input = input_
        self._last_time = now

        return output
