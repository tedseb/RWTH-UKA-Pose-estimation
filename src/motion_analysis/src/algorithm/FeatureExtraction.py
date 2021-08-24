#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains code that deals with the  manual extraction of low-level features from skelletons.
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


import math
from abc import abstractmethod
from itertools import combinations
from typing import List, Tuple

import hashlib
import numpy as np

try:
    from motion_analysis.algorithm.config import *
    from motion_analysis.algorithm.AlgoUtils import *
except ImportError:
    from algorithm.config import *
    from algorithm.AlgoUtils import *

from scipy.linalg import hankel


class FeatureExtractorException(Exception):
    pass

class UnknownAngleException(FeatureExtractorException):
    pass


def create_vector_from_two_points(a, b):
        return np.array([b[X] - a[X], b[Y] - a[Y], b[Z] - a[Z]])

def dot_product(a, b):
    return a[X] * b[X] + a[Y] * b[Y] + a[Z] * b[Z]

def length_of_vector(x):
    return math.sqrt(math.pow(x[X], 2) + math.pow(x[Y], 2) + math.pow(x[Z], 2))


def discritize_feature_values(feature_value, last_resampled_feature_value, resolution):
    new_resampled_feature_values = []
    delta = feature_value - last_resampled_feature_value
    remaining_delta = abs(delta)
    
    while remaining_delta >= resolution:
        remaining_delta -= resolution
        last_resampled_feature_value += math.copysign(resolution, delta)
        new_resampled_feature_values.append(last_resampled_feature_value)

    return new_resampled_feature_values


def extract_average_height_of_joints(pose_array: np.ndarray) -> float:
    """Compute the height of a person as the average of the height of its joints.
    
    Args:
        pose_array: A pose, according to the ROS Message 'Person'.
        
    Returns:
        A float value corresponding to the height of the person.
    """
    height = 0
    for pose in pose_array:
        height += pose[Z]
    
    return height

def extract_height_of_joint(pose_array: np.ndarray, index: int) -> float:
    """Extract the height of a joint.
    
    Args:
        pose_array: A pose, according to the ROS Message 'Person'.
        
    Returns:
        A float value corresponding to the height of the joint
    """

    return pose_array[index][Z]

def extract_height_of_body_core(pose_array: np.ndarray, pose_definition_adapter: PoseDefinitionAdapter) -> float:
    """Compute the average height of pelvis and neck, representing the height of the body's core.
    
    Args:
        pose_array: A pose, according to the ROS Message 'Person'.

    Returns:
    A float value corresponding to the height of the body's core.
    """
    
    neck_idx = pose_definition_adapter.get_joint_index("OP_Neck")
    pelvis_idx = pose_definition_adapter.get_joint_index("Pelvis_MPII")

    return np.average([pose_array[neck_idx][Z], pose_array[pelvis_idx][Z]])
    
    
def extract_distance(pose_array: np.ndarray, joint_a: int, joint_b: int) -> float:
    """Compute the distance between two joints.

    Args:
        pose: A pose, according to the ROS Message 'Person'
        joint_a: Index of one of two joints between we measure the distance
        joint_b: Name of the the second joint to measure the distance

    Returns:
        A float value corresponding to the euclidean distance between joint_a and joint_b.
    """
    vector = create_vector_from_two_points(pose_array[joint_a], pose_array[joint_b])

    return length_of_vector(vector)

def extract_angle(pose_array: np.ndarray, inner_joint: int, outer_joints: set) -> float:
    """Compute the angle between three connected joints.
    
    Args:
        pose: A pose, according to the ROS Message 'Person'
        innter_joint: Index of the joint connected to both of the outer joints
        outer_joints: A set of two outer joint's indices

    Returns:
        A float value corresponding to the inner angle between the three defined joints.
    """

    ba = create_vector_from_two_points(pose_array[inner_joint], pose_array[outer_joints[0]])
    bc = create_vector_from_two_points(pose_array[inner_joint], pose_array[outer_joints[1]])
    x = dot_product(ba, bc) / (length_of_vector(ba) * length_of_vector(bc))

    angle = math.acos(min(max(x, -1), 1)) * 180 / math.pi
    return angle


def extract_rotation(pose_array: np.ndarray, joint: int) -> float:
    """Compute the rotation of a joint.

    Args:
        pose: A pose, according to the ROS Message 'Person'
        joint: Index of the joint

    Returns:
        TODO: Specifiy our understanding of rotation (relative to what?)
    """

    raise NotImplementedError("Currently not part of the features of interest.")


def extract_speed(poses_array: np.ndarray, joint: int) -> float:
    """Compute the speed of a joint, averaged over a list of poses.

    Args:
        poses: A list of poses, according to the ROS Message 'Person'
        joint: Index of the joint

    Returns:
        A float value corresponding to the speed of the joint, averaged over the list of poses.
    """
    raise NotImplementedError("Currently not part of the features of interest.")


def extract_acceleration(poses_array: np.ndarray, joint: int) -> float:
    """Compute the acceleration of a joint, averaged over a list of poses.

    Args:
        poses: A list of poses, according to the ROS Message 'Person'
        joint: Index of the joint

    Returns:
        A float value corresponding to the acceleration of the joint, averaged over the list of poses. 
    """
    raise NotImplementedError("Currently not part of the features of interest.")


def extract_feature_of_interest_specification_dictionary(exercise_data: dict, pose_definition_adapter: PoseDefinitionAdapter) -> dict:
    """This method turns an exercise data dictionary into a dictionary of features.

    Features are specified according to the following structure:
    { # one entry per feature type
        'angles': <list of angles>,
        (...)
    }
    """
    features_of_interest = dict()
    for feature_name, extraction_method in [('angles', extract_angles_of_interest)]:
        features_of_interest[feature_name] = extraction_method(exercise_data, pose_definition_adapter)

    return features_of_interest


def extract_angles_of_interest(exercise_data: dict, pose_definition_adapter: PoseDefinitionAdapter) -> dict:
    """A triplet of three joints has three angles. This method finds the inner and outer joints for an angle.

    The inner and outer joints dictionary returned by this method has the following form:

    {
        <frozenset of joint name strings>: {
            "inner_joint": <inner joint>
            "outer_joints": <set of outer joints>
        },
        (...)
    }

    Args:
        exercise_data: The exercise data that hold information on what angles the expert chose.

    Returns:
        A dictionary of inner joints and sets of outer joints.
    """
    def find_inner_and_outer_joints(joint_names: Tuple[str, str, str]) -> Tuple[str, Tuple[str, str]]:
        """Take 3 joints and look their connections up in pose_definition_adapter.joint_connection_labels to see if there is an inner joint.
        
        Args:
            joint_names: 
        """
        if not len(joint_names) == 3:
            raise FeatureExtractorException("Can not calculate angle between other than 3 points.")
        
        joint_connection_combinations = combinations(joint_names, 2)

        # We must find two connections between our three joints, such that there is a usefull angle between them
        combinations_found = 0

        for joint_connection_combination in joint_connection_combinations:
            if frozenset(joint_connection_combination) not in pose_definition_adapter.joint_connections_labels:
                outer_joints = joint_connection_combination
                inner_joint, *_ = joint_names.difference(joint_connection_combination)
            else:
                combinations_found += 1

        if combinations_found == 2:
            return inner_joint, outer_joints
        else:
            raise UnknownAngleException("Specified angle between joints that are not connected")

    
    # Before the beta, Tamer defines exercise data in stages. This method will be here only for a couple of weeks.
    joints_of_interest = set()
    # TODO: This needs to be changed as soon as Tamer stops sending stages but sends angles of interest
    for stage in exercise_data['stages']:
        for rule_joints in stage['angles']:
            joints_of_interest.update({frozenset(rule_joints)})

    exceptions = dict()
    inner_and_outer_joints = {}
    for joint_names in joints_of_interest:
        try:
            inner_joint, outer_joints = find_inner_and_outer_joints(joint_names)
        except UnknownAngleException as e:
            exceptions['UnknownAngleException'] = e
            continue
        except FeatureExtractorException as e:
            exceptions['FeatureExtractorException'] = e
        # TODO: possibly find prettier solution to this
        angle_hash = str(hashlib.md5(sorted(joint_names).__repr__().encode()).digest())
        inner_and_outer_joints[angle_hash] = {"inner_joint": inner_joint, "outer_joints": outer_joints}
    
    if exceptions and HIGH_VERBOSITY:
        rp.logerr("Errors occured while parsing the provided exercise:" + str(exceptions))

    return inner_and_outer_joints


def extract_states(pose_array: np.ndarray, last_feature_states: dict, reference_feature_data: dict, feature_of_interest_specification: dict, pose_definition_adapter: PoseDefinitionAdapter):
    """Extract the states.

    Args:
        pose_array: An array representing a human pose.
        reference_feature_data: A dictionary containing data on to features to be extracted, taken from the reference trajectory
        feature_of_interest_specification: A dictionary containing data on what state to extract
    
    In order to so, we take a naive approach and test how many of the angles of interest in the first step of the recording tend towards to higher boundary.
    """
    state_dict = dict()

    def create_feature_dict(value, last_feature_state, lower_boundary, upper_boundary):
        feature_dict = {"feature_value": value}

        feature_dict["feature_state"] = decide_feature_state(value, last_feature_state, lower_boundary, upper_boundary)

        return feature_dict


    for feature_type, feature_specification in feature_of_interest_specification.items():
        state_dict[feature_type] = dict()
        if feature_type == 'angles':
            # Subroutine that calculates the boundaries for angles
            for joint_hash, features in feature_specification.items():
                feature_key = joint_hash
                state_dict[feature_type][joint_hash] = dict()
                inner_joint_name, outer_joints_names = features["inner_joint"], features["outer_joints"]
                inner_joint_idx = pose_definition_adapter.get_joint_index(inner_joint_name)
                outer_joint_idxs = tuple(pose_definition_adapter.get_joint_index(n) for n in outer_joints_names)

                value = extract_angle(pose_array, inner_joint_idx, outer_joint_idxs)

                if last_feature_states:
                    last_feature_state = last_feature_states["angles"][joint_hash]
                else:
                    last_feature_state = None

                state_dict[feature_type][feature_key] = create_feature_dict(value, last_feature_state, \
                    reference_feature_data["angles"][joint_hash]["lower_boundary"], \
                        reference_feature_data["angles"][joint_hash]["upper_boundary"])
        else:
            raise NotImplementedError("Trying to extract states for an unspecified feature type")
        
    return state_dict


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
        return FEATURE_HIGH
    
    if value <= lower_boundary:
        return FEATURE_LOW
    
    if last_feature_state == FEATURE_HIGH:
        return FEATURE_HIGH_UNDECIDED
    
    if last_feature_state == FEATURE_LOW:
        return FEATURE_LOW_UNDECIDED

    return FEATURE_UNDECIDED


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
            already_discritized_values = discritize_feature_values(value, last_values[-1], resolution)
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

    return discrete_trajectories_tensor, discretization_reference_trajectory_indices_tensor, hankel_tensor, feature_states_matrix, scale


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


def extract_reference_feature_data_from_feature_trajectories(feature_trajectories: dict, recording_lengths: list) -> dict:
    """Extract the boundaries of angles (and in the future possibly distances).
        
    This method takes a dict consisting of feature trajectories and turns these trajectories into boudnaries
    that take the places of the trajectories in the dict.

    Args:
        feature_trajectories: A nested dictionary which looks as follows:
        { # One entry for each feature type
            <feature type>: <list of feature lists>,
            (...)
        }
    
    Return:
        A dictionary containing extracted information about a feature
    """

    lowest_values = []
    highest_values = []
    # The values here are each lists of feature values of pose trajectories
    for trajectory in feature_trajectories:
        lowest_values.append(np.amin(trajectory))
        highest_values.append(np.amax(trajectory))

    # We take the average of highest and lowest values to compute the boundaries
    highest_value = np.average(highest_values)
    lowest_value = np.average(lowest_values)

    # We then compute the boundaries as the range of motion of reference tractories, with tolerances
    range_of_motion = abs(highest_value - lowest_value)
    lower_boundary = lowest_value + range_of_motion * REDUCED_RANGE_OF_MOTION_TOLERANCE_LOWER
    upper_boundary = highest_value - range_of_motion * REDUCED_RANGE_OF_MOTION_TOLERANCE_HIGHER

    discrete_trajectories_tensor, discretization_reference_trajectory_indices_tensor, hankel_tensor, feature_states_matrix, scale = compute_discrete_trajectoreis_hankel_matrices_and_feature_states(feature_trajectories, range_of_motion, lower_boundary, upper_boundary)

    # TODO: Maybe do this for every feature trajectory separately and take the median of these as the number of state changes
    median_feature_states_array = compute_median_feature_states(feature_states_matrix)

    number_of_dicided_state_changes = compute_numer_of_dicided_state_changes(median_feature_states_array)

    median_reference_trajectory, median_reference_trajectory_feature_states, median_reference_trajectory_discretization_ranges = compute_median_discrete_trajectory_median_feature_states_and_reference_trajectory_fractions(discrete_trajectories_tensor, discretization_reference_trajectory_indices_tensor, lower_boundary, upper_boundary, recording_lengths)
    # TODO: Check wether median_reference_trajectory_feature_states match the median state trajectory

    return {"lower_boundary": lower_boundary, \
        "upper_boundary": upper_boundary, \
            "lowest_value": lowest_value, \
                "highest_value": highest_value, \
                    "range_of_motion": range_of_motion, \
                        "discretization_reference_trajectory_indices_tensor": discretization_reference_trajectory_indices_tensor, \
                            "hankel_tensor": hankel_tensor, \
                                "scale": scale, \
                                    "median_trajectory": median_reference_trajectory, \
                                        "median_reference_trajectory_feature_states": median_reference_trajectory_feature_states, \
                                            "median_resampled_values_reference_trajectory_fractions": median_reference_trajectory_discretization_ranges,
                                                "number_of_changes_in_decided_feature_states": number_of_dicided_state_changes}


def extract_reference_feature_data_from_recordings(recordings: List[np.ndarray], feature_of_interest_specification: dict, pose_definition_adapter: PoseDefinitionAdapter) -> dict:
    """Extract the trajectories of angles (and in the future possibly distances and other features).
    
    Under consideration of the specification of featuers of interest, this method extracts trajectories
    of the specified features from a recording. The feature_of_interest_specification is then returned
    with the boundaries added.

    Args:
        recording: An ndarray containing a sequence of ndarrays that each represent a pose.
        feature_if_interest_specification: A nested dictionary which looks as follows:
        { # One entry for each feature type
            <feature type>: <feature specification>,
            (...)
        }
        pose_definition_adater: A PoseDefinitionAdapter
    
    Return:
        The boundary specification, which follows the same structure as the features_of_interest_specification.
    """
    reference_feature_data_trajectory_dict = dict()

    recording_lengths = []
    for recording in recordings:
        recording_lengths.append(len(recording))
    
    for feature_type, feature_specification in feature_of_interest_specification.items():
        reference_feature_data_trajectory_dict[feature_type] = dict()
        if feature_type == 'angles':
            for joint_hash, features in feature_specification.items():
                reference_feature_data_trajectory_dict[feature_type][joint_hash] = dict()
                inner_joint_name, outer_joints_names = features["inner_joint"], features["outer_joints"]
                inner_joint_idx = pose_definition_adapter.get_joint_index(inner_joint_name)
                outer_joint_idxs = tuple(pose_definition_adapter.get_joint_index(n) for n in outer_joints_names)
                recording_angles_list = []
                for recording in recordings:
                    recording_angles = []
                    for pose_array in recording:
                        recording_angles.append(extract_angle(pose_array, inner_joint_idx, outer_joint_idxs))
                    recording_angles_list.append(recording_angles)
                
                reference_feature_data_trajectory_dict[feature_type][joint_hash] = extract_reference_feature_data_from_feature_trajectories(recording_angles_list, recording_lengths)
        else:
            raise NotImplementedError("Trying to extract boundaries for an unspecified feature type")

    return reference_feature_data_trajectory_dict


def map_progress_to_vector(progress: float):
    """ Calculate the cartesian represenation of a progress as a unit vector.
    
    We specift a progress as a number in the range (0...1).
    We interpret 0 and 1 as the same progress, i.e. beginning and end of a repetition.
    Hence, the progress can be interpreted as an angle between 0 and 2*PI.

    Args:
        progress: A float value in the range (0...1), representing the progress in a repetition, as indicated by a feature.
    
    Return:
        An np.array containing x and y values of the progress.
    """
    return complex(np.cos(progress*2*np.pi), np.sin(progress*2*np.pi))

def map_vectors_to_progress_and_alignment(vectors: list):
    """ Sum a list of 2D vectors up to a single vector, representing the overall, averaged progress.

    The avaraged progress has a direction and therefore an angle.
    We use this angle to calculate the overall progress.
    The length of overall progress vector, divided by the number of vectors is a measurement for its alignment.

    Args:
        vectors: A list of progress vectors

    Return:
        The angle of the overall progress, the alignment of the vectors and the progress alignment vector
    
    """
    progress_vector_sum = sum(vectors)
    # rp.logerr(vectors)
    # rp.logerr(progress_vector_sum)
    progress = np.angle(progress_vector_sum) + (2 * np.pi)
    progress = progress % (2 * np.pi) # Bring progress back our notation of a value in range (0...1)
    # rp.logerr(progress)
    progress = progress / (2 * np.pi)
    alignment = np.abs(progress_vector_sum) / len(vectors)
    progress_alignment_vector = progress_vector_sum / len(vectors)

    return  progress, alignment, progress_alignment_vector