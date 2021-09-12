#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains features.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
At the time of writing, such features are explicitly angles. Other features might follow in the future.

TODO: Use msgpack at Shawans end do extract features there already? https://pypi.org/project/msgpack-numpy/

We support the following features that can be extracted either from single skeletons or series of skeletons:
    * Distance      (between 2 joints)      :scalar
    * Angle         (between 3 joints)      :scalar
    * Rotation      (of 1 joint)            :vector
    * Speed         (of 1 joint)            :vector
    * Acceleration  (of 1 joint)            :vector


We define our numpy arrays representing skelletons as follows:
[ # joints 0... n
    [ # x, y, z
        x, # float16
        y, # float16 # z - axis in the SPIN coordinates
        z, # float16 # y - axis in the SPIN coordinates
    
    (...)
]
"""

import collections
from enum import IntEnum
from collections import deque
import numpy as np
import math
from abc import ABC
from scipy.linalg import hankel
from typing import List

try:
    from motion_analysis.src.algorithm.AlgoConfig import *
    from motion_analysis.src.algorithm.AlgoUtils import *
except (ModuleNotFoundError, ImportError):
    from src.algorithm.AlgoConfig import *
    from src.algorithm.AlgoUtils import *


class FeatureState(IntEnum):
    # Alias for features states
    FEATURE_LOW: int = -2
    FEATURE_LOW_UNDECIDED = -1
    FEATURE_UNDECIDED: int = 0
    FEATURE_HIGH_UNDECIDED = 1
    FEATURE_HIGH: int = 2


class FeatureType(IntEnum):
    ANGLE = 0
    ANGULAR_SPEED = 1
    ANGULAR_ACCELERATION = 2
    JOINT_DISTANCE = 3
    JOINT_ACCELERATION = 4


class BaseFeature(ABC):
    def __init__(self, \
        feature_hash: str, \
            specification_dict: object = {}):
            
        self.feature_hash = feature_hash
        self.type = specification_dict["type"]
        # If there is anything else apart from the feature_hash to specify what feature this is, we specify it in this dict
        self.specification_dict = specification_dict
        
        from src.algorithm.FeatureExtraction import feature_extraction_methods # Try to remove this
        self.feature_extraction_method = feature_extraction_methods[self.type]

    @property
    def value(self):
        return self.values[-1]

    @property
    def resampled_value(self):
        return self.resampled_values[-1]

    @property
    def state(self):
        return self.states[-1]


class ReferenceFeature(BaseFeature):
    def __init__(self, \
        feature_hash: str, \
            specification_dict: object = {}):
        super().__init__(feature_hash, specification_dict)

        self.values = deque()
        self.progression = 0
        self.resampled_values = deque()
        self.states = deque()

        self.lower_boundary = None
        self.upper_boundary = None
        self.range_of_motion = None 

        self.resolution = None
        self.scale = None

    def add_pose(self, pose: np.ndarray, pose_definition_adapter: PoseDefinitionAdapter):
        value = self.feature_extraction_method(pose, self.specification_dict, pose_definition_adapter)
        self.values.append(value)

    def update_data(self):
        self.highest_value = np.amax(self.values)
        self.lowest_value = np.amin(self.values)
        self.recording_length = len(self.values)

        # We then compute the boundaries as the range of motion of reference tractories, with tolerances
        self.range_of_motion = abs(self.highest_value - self.lowest_value)
        self.lower_boundary = self.lowest_value + self.range_of_motion * REDUCED_RANGE_OF_MOTION_TOLERANCE_LOWER
        self.upper_boundary = self.highest_value - self.range_of_motion * REDUCED_RANGE_OF_MOTION_TOLERANCE_HIGHER
        
        first_reference_feature_value = self.values[0]
        self.beginning_state = decide_feature_state(first_reference_feature_value, FeatureState.FEATURE_UNDECIDED, self.lower_boundary, self.upper_boundary)
    

class ReferenceFeatureCollection(BaseFeature):
    def __init__(self, \
        feature_hash: str, \
            specification_dict: object = {}, \
                recordings_and_adapters: List[np.ndarray] = []):

        super().__init__(feature_hash, specification_dict)
        self.reference_features = list()

        for (recording, adapter) in recordings_and_adapters:
            self.add_recording(recording, adapter)

        self.update_data()
        
    def add_recording(self, recording: np.ndarray, pose_definition_adapter):
        reference_feature = ReferenceFeature(self.feature_hash, self.specification_dict)
        for pose in recording:
            # For now, we use the same pose_defninition_adapter for all recordings. This may change
            reference_feature.add_pose(pose, pose_definition_adapter)
        reference_feature.update_data()
        self.reference_features.append(reference_feature)

    def update_data(self):
        lowest_values = []
        highest_values = []
        recording_lengths = []
        # The values here are each lists of feature values of pose trajectories
        trajectories = [rf.values for rf in self.reference_features]
        for trajectory in trajectories:
            lowest_values.append(np.amin(trajectory))
            highest_values.append(np.amax(trajectory))
            recording_lengths.append(len(trajectory))

        # We take the average of highest and lowest values to compute the boundaries
        self.highest_value = np.average(highest_values)
        self.lowest_value = np.average(lowest_values)

        # We then compute the boundaries as the range of motion of reference tractories, with tolerances
        self.range_of_motion = abs(self.highest_value - self.lowest_value)
        self.lower_boundary = self.lowest_value + self.range_of_motion * REDUCED_RANGE_OF_MOTION_TOLERANCE_LOWER
        self.upper_boundary = self.highest_value - self.range_of_motion * REDUCED_RANGE_OF_MOTION_TOLERANCE_HIGHER

        discrete_trajectories_tensor, \
            discretization_reference_trajectory_indices_tensor, \
                self.hankel_tensor, \
                    feature_states_matrix, \
                        self.scale, \
                            self.resolution = compute_discrete_trajectoreis_hankel_matrices_and_feature_states(trajectories, self.range_of_motion, self.lower_boundary, self.upper_boundary)

        # TODO: Maybe do this for every feature trajectory separately and take the median of these as the number of state changes
        median_feature_states_array = compute_median_feature_states(feature_states_matrix)

        self.number_of_dicided_state_changes = compute_numer_of_dicided_state_changes(median_feature_states_array)

        self.median_trajectory, self.median_trajectory_feature_states, self.median_trajectory_discretization_ranges = compute_median_discrete_trajectory_median_feature_states_and_reference_trajectory_fractions(discrete_trajectories_tensor, discretization_reference_trajectory_indices_tensor, self.lower_boundary, self.upper_boundary, recording_lengths)
        # TODO: Check wether median_reference_trajectory_feature_states match the median state trajectory

        beginning_states = [r.beginning_state for r in self.reference_features]
        self.median_beginning_state = np.median(beginning_states)


class Feature(BaseFeature):
    def __init__(self, reference_feature_collection: ReferenceFeatureCollection, max_trajectory_length=FEATURE_TRAJECTORY_MAX_MEMORY_SIZE):

        super().__init__(reference_feature_collection.feature_hash, reference_feature_collection.specification_dict)
        self.reference_feature_collection = reference_feature_collection
        self.lower_boundary = reference_feature_collection.lower_boundary
        self.upper_boundary = reference_feature_collection.upper_boundary
        self.resolution = reference_feature_collection.resolution

        self.values = deque(maxlen=max_trajectory_length)
        self.progression = 0
        self.discretized_values = deque(maxlen=max_trajectory_length)
        self.states = deque(maxlen=max_trajectory_length)

    def update(self, pose: np.ndarray, pose_definition_adapter: PoseDefinitionAdapter):
        value = self.feature_extraction_method(pose, self.specification_dict, pose_definition_adapter)

        try:
            discretized_values =  discretize_feature_values(value, self.value, self.resolution)
            if discretized_values:
                self.discretized_values.extend(discretized_values)
        except IndexError:
            scale = self.reference_feature_collection.scale
            # If this is the first value, extend with the nearest value on our scale
            self.discretized_values.extend(scale[np.argmin(abs(scale - value))])

        self.values.append(value)
        self.compute_new_feature_progression()
        self.states.append(decide_feature_state(value, self.state, self.lower_boundary, self.upper_boundary))


    def compute_new_feature_progression(self):
        """Compute a dictionary representing the progression of the features specified by feature_state

        This method turns features states, such as FEATURE_HIGH or FEATURE_LOW into a feature progression,
        that is a number that represents the number of state changes in this feature in this repetition, 
        depending on the previous progression of the feature. 
        This way we can track the progression of different features between timesteps.

        Args: 
            beginning_state: A dictionary that holds the state in which a feature begins for every feature of every category
            features_state: The state that the featuers are in
            last_feature_progression: The last dictionary produced by this method in the last timestep

        Return:
            new_feature_progression: The feature progression dictionary at this timestep

        Raises:
            MalformedFeatures: If features are not the expected form.
        """
        # If features beginn with the FEATURE_HIGH state, feature progressions must be odd if the feature state changes to FEATURE_LOW and even afterwards
        if self.reference_feature_collection.median_beginning_state == FeatureState.FEATURE_HIGH and \
            ((self.state <= FeatureState.FEATURE_LOW_UNDECIDED and self.progression % 2 == 0) or \
                (self.state >= FeatureState.FEATURE_HIGH_UNDECIDED and self.progression % 2 == 1)):
                new_feature_progression = self.progression + 1
        # If features beginn with the FEATURE_LOW state, feature progressions must be odd if the feature state changes to FEATURE_HIGH and even afterwards
        elif self.reference_feature_collection.median_beginning_state == FeatureState.FEATURE_LOW and \
            ((self.state >= FeatureState.FEATURE_HIGH_UNDECIDED and self.progression % 2 == 0) or \
                (self.state <= FeatureState.FEATURE_LOW_UNDECIDED and self.progression % 2 == 1)):
                new_feature_progression = self.progression + 1
        else:
            new_feature_progression = self.progression

        self.progression = new_feature_progression


def enqueue_dictionary(previous_dict, enqueued_dict, max_queue_size):
    for k, v in enqueued_dict.items():
        if isinstance(v, collections.MutableMapping):
            previous_dict[k] = enqueue_dictionary(previous_dict.get(k, {}), v, max_queue_size)
        elif isinstance(v, list):
            previous_dict[k] = previous_dict.get(k, [])
            previous_dict[k].extend(v)
            if (len(previous_dict[k]) >= max_queue_size):
                previous_dict[k] = previous_dict[k][-max_queue_size:]
        else:
            previous_dict[k] = previous_dict.get(k, [])
            if previous_dict[k] == None:
                previous_dict[k] = []
            previous_dict[k].append(v)
            if (len(previous_dict[k]) >= max_queue_size):
                previous_dict[k] = previous_dict[k][-max_queue_size:]
    
    return previous_dict


def discretize_feature_values(value, last_discritized_value, resolution):
    new_resampled_feature_values = []
    delta = value - last_discritized_value
    remaining_delta = abs(delta)
    
    while remaining_delta >= resolution:
        remaining_delta -= resolution
        last_discritized_value += math.copysign(resolution, delta)
        new_resampled_feature_values.append(last_discritized_value)

    return new_resampled_feature_values


def remove_jitter_from_trajectory(trajectory, _range):
    """Remove repeated ups and downs in a trajectory.
    
    A trajectory may be subjected to quick up and down movement as unreliable measurements are taken.
    For example measurements of an angle can jitter if the adjecent joints jitter.
    With this method we remove such measurements from a trajectory.
    
    Args:
        trajectory: A list of values that we want to free from jitter.
        _range: The range of values that can be part of the jitter. i.e. 3 ups and 3 downs is a range of 3.
        
    Return:
        The trimmed trajectory
    """
    done = False
    while not done:
        done = True
        try:
            for i in range(1, _range):
                for j in range(i, len(trajectory) - 3 * i):
                    window = trajectory[j: j + 3 * i]
                    if np.array_equal(window[:i], window[2 * i:]):
                        done = False
                        try:
                            del(trajectory[j: j + 3 * i])
                        except ValueError:
                            trajectory = np.delete(trajectory, range(j, j + 3 * i - 1))
                            raise ContinueToOuterLoop
        except ContinueToOuterLoop:
            pass
                        
    return trajectory


def compute_discrete_trajectoreis_hankel_matrices_and_feature_states(feature_trajectories, range_of_motion, lower_boundary, upper_boundary):
    """Discritize a list of trajectories and compute their hankel tensor and features states.

    Args:
        feature_trajectories: A list of trajectories corresponding to multiple recordings of the same feature.
        range_of_motion: A float representing how much the feature varies in the recordings
        lower_boundary: The boundary below which a feature state counts as 'low'
        lower_boundary: The boundary above which a feature state counts as 'high'

    Return:
        A tensor containing the discretized versions of the given trajectories
        Indices of the feature values per discrete_trajectories_tensor
        The hankel tensor that corresponds to the discrete trajectories
        A matrix of features states obtained per discrete feature value
        The scale that the discrete_trajectories_tensor was constructed with
    """
    # We compute the resampled values separately for each trajectory and turn them into hankel matrices
    resolution = range_of_motion * FEATURE_TRAJECTORY_RESOLUTION_FACTOR
    discrete_trajectories_tensor = []
    discretization_reference_trajectory_indices_tensor = [] # For every resampled value, we need an index to point us to the original pose
    hankel_tensor = [] # In the comparing algorithm, we need a hankel matrix of every resampled reference trajectory
    feature_states_matrix = list()
    for trajectory in feature_trajectories:
        last_values = [trajectory[X]]
        last_feature_state = decide_feature_state(last_values[-1], None, lower_boundary, upper_boundary)
        discrete_values = last_values
        feature_trajectory_indices = [X]
        feature_states = list()
        for index, value in enumerate(trajectory):
            already_discritized_values = discretize_feature_values(value, last_values[-1], resolution)
            discrete_values.extend(already_discritized_values)
            if already_discritized_values:
                feature_state = decide_feature_state(already_discritized_values[-1], last_feature_state, lower_boundary, upper_boundary)
                last_values = already_discritized_values
                feature_trajectory_indices.extend([index] * len(last_values))
                if feature_state != last_feature_state:
                    feature_states.append(feature_state)
                    last_feature_state = feature_state

        discrete_values = trajectory = remove_jitter_from_trajectory(discrete_values, REMOVE_JITTER_RANGE)
        discrete_trajectories_tensor.append(discrete_values)
        hankel_matrix = hankel(discrete_trajectories_tensor, np.roll(discrete_trajectories_tensor, -1))
        hankel_matrix = np.roll(hankel_matrix, -1, axis=0) # The first column of the hankel matrix represents the last frame of the recording
        hankel_tensor.append(hankel_matrix)
        discretization_reference_trajectory_indices_tensor.append(feature_trajectory_indices)
        feature_states_matrix.append(feature_states)

    hankel_tensor = np.asarray(hankel_tensor, dtype=np.float16)
    feature_states_matrix = np.asarray(feature_states_matrix, dtype=np.int8)
    discretization_reference_trajectory_indices_tensor = np.asarray(discretization_reference_trajectory_indices_tensor, dtype=np.int16)

    scale = set()
    for t in discrete_trajectories_tensor:
        scale.update(set(t))

    scale = np.sort(list(scale))
    #TODO: Turn usage of this scale into a np.digitize(value, bins=scale) -type of difitization?

    return discrete_trajectories_tensor, discretization_reference_trajectory_indices_tensor, hankel_tensor, feature_states_matrix, scale, resolution


def compute_median_feature_states(feature_states_matrix):
    """Compute the median of feature states for every timestep.

    Args:
        feature_states_matrix: A matrix of multiple rows of feature state trajectories.

    Return:
        An array representing the median of feature states.
    """
    median_feature_states = list()
    median_length = np.int(np.median([len(states) for states in feature_states_matrix]))
    for i in range(median_length):
        median_feature_state = np.median(feature_states_matrix[:, i])
        while not np.where(feature_states_matrix[:, i] != median_feature_state):
            bad_feature_state_indices = np.where(feature_states_matrix[:, i] != median_feature_state)
            # Modify trajectories that differ such that they stay useful
            for bad_feature_state_index in bad_feature_state_indices:
                if len(feature_states_matrix[:, i][bad_feature_state_index]) > median_length:
                    # We cut parts of longer state trajectories
                    np.delete(feature_states_matrix[:, i], bad_feature_state_index)
                elif len(feature_states_matrix[:, i][bad_feature_state_index]) == median_length:
                    # We change states in trajectories of adequate length
                    feature_states_matrix[bad_feature_state_index, i] = median_feature_state
                else:
                    # We add dummy parts to shorter state trajectories
                    np.insert(feature_states_matrix[:, i], bad_feature_state_index, median_feature_state)
        median_feature_states.append(median_feature_state)
    
    return np.array(median_feature_states)


def compute_median_discrete_trajectory_median_feature_states_and_reference_trajectory_fractions(discrete_trajectories_tensor, discretization_reference_trajectory_indices_tensor, lower_boundary, upper_boundary, recording_lengths):
    """Compute the median of multiple trajectories per value, the median of the feature states and the reference trajectory fractions.

    Args:
        discrete_trajectories_tensor: A tensor representing the discrete trajectories gained from examining feature trajectories.
        discretization_reference_trajectory_indices_tensor: The reference trajectory indices at which the discrete values where computed
        lower_boundary: The boundary below which a feature state counts as 'low'
        lower_boundary: The boundary above which a feature state counts as 'high'
        recording_lengths: length of the original recording

    Return:
        The median discrete trajectory
        The median feature states
        The discretization ranges
    """
    # Construction of a median feature trajectory
    discretization_ranges = list() # Tells us where in the overall reference trajectory this resampled median value lies as a fraction
    all_feature_values_array = np.array(discrete_trajectories_tensor)
    median_trajectory = list()
    median_length = np.int(np.median([len(values) for values in discrete_trajectories_tensor]))
    for i in range(median_length):
        # rp.logerr(discrete_trajectories_tensor)
        # rp.logerr(discretization_reference_trajectory_indices_tensor)
        # rp.logerr(recording_lengths)
        median_resampled_values_reference_trajectory_fraction_from = np.average([discretization_reference_trajectory_indices_tensor[j, i]/recording_lengths[j] for j in range(len(recording_lengths))])
        median_resampled_values_reference_trajectory_fraction_to = np.average([discretization_reference_trajectory_indices_tensor[j, (i + 1) % len(discretization_reference_trajectory_indices_tensor[j])]/recording_lengths[j] for j in range(len(recording_lengths))])
        median_feature_value = np.median(all_feature_values_array[:, i])
        while not np.where(all_feature_values_array[:, i] != median_feature_value):
            bad_feature_state_indices = np.where(all_feature_values_array[:, i] != median_feature_value)
            # Modify trajectories that differ such that they stay useful
            for bad_feature_state_index in bad_feature_state_indices:
                if len(all_feature_values_array[:, i][bad_feature_state_index]) > median_length:
                    # We cut parts of longer trajectories
                    np.delete(all_feature_values_array[:, i], bad_feature_state_index)
                elif len(all_feature_values_array[:, i][bad_feature_state_index]) == median_length:
                    # We change values in trajectories of adequate length
                    all_feature_values_array[bad_feature_state_index, i] = median_feature_value
                else:
                    # We add "dummy" parts to shorter trajectories
                    np.insert(all_feature_values_array[:, i], bad_feature_state_index, median_feature_value)
        median_trajectory.append(median_feature_value)
        discretization_ranges.append({"median_resampled_values_reference_trajectory_fraction_from": median_resampled_values_reference_trajectory_fraction_from, \
            "median_resampled_values_reference_trajectory_fraction_to": median_resampled_values_reference_trajectory_fraction_to})

    # Shorten median_reference_trajectory, if values double
    last_value = np.inf
    feature_states = list()
    last_feature_state = None
    for index, value in enumerate(median_trajectory):
        feature_state = decide_feature_state(value, last_feature_state, lower_boundary, upper_boundary)
        last_feature_state = feature_state
        if value == last_value:
            np.delete(median_trajectory, index)
            continue
        last_value = value
        feature_states.append(feature_state)

    return median_trajectory, feature_states, discretization_ranges


def compute_numer_of_dicided_state_changes(median_feature_states_array):
    decided_median_feature_states = median_feature_states_array[abs(median_feature_states_array) > 1]
    decided_median_feature_state_change_indices = np.where(decided_median_feature_states[:-1] * decided_median_feature_states[1:] < 0 )[X] + 1
    number_of_dicided_state_changes = len(decided_median_feature_state_change_indices)
    if decided_median_feature_states[X] != decided_median_feature_states[-1]:
        number_of_dicided_state_changes += 1
    return number_of_dicided_state_changes


def decide_feature_state(value, last_feature_state, lower_boundary, upper_boundary):
    """Compute the state of a feature.
    
    Args:
        value: The value of feature
        lower_boundary: The boundary below which a feature state counts as 'low'
        lower_boundary: The boundary above which a feature state counts as 'high'
        
    Return:
        feature_state: The state of the feature, expressed as an integer between -1 and 1
    """
    if value >= upper_boundary:
        return FeatureState.FEATURE_HIGH
    
    if value <= lower_boundary:
        return FeatureState.FEATURE_LOW
    
    if last_feature_state == FeatureState.FEATURE_HIGH:
        return FeatureState.FEATURE_HIGH_UNDECIDED
    
    if last_feature_state == FeatureState.FEATURE_LOW:
        return FeatureState.FEATURE_LOW_UNDECIDED

    return FeatureState.FEATURE_UNDECIDED
