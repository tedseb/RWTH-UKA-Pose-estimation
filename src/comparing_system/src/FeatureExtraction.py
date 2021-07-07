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
from abc import ABC, abstractmethod
from itertools import combinations
from typing import Any, Dict, List, Tuple

import hashlib
import collections
import msgpack
import numpy as np
import rospy as rp
import scipy
try:
    from comparing_system.src.config import *
    from comparing_system.src.Util import *
except ImportError:
    from src.config import *
    from src.Util import *

from scipy.linalg import hankel
from scipy.spatial.distance import euclidean
from backend.msg import Bodypart


class FeatureExtractorException(Exception):
    pass

class UnknownAngleException(FeatureExtractorException):
    pass


class PoseDefinitionAdapter():
    @abstractmethod
    def __init__():
        pass

    @abstractmethod
    def get_joint_index(self, joint_name: str) -> int:
        """Get the index of a joint from its name"""
        raise NotImplementedError("This is an interface, it should not be called directly.")

    @abstractmethod
    def body_parts_to_ndarray(self, body_parts: Bodypart) -> np.ndarray:
        """Take a Bodyparts object, defined by the backend messages turn it into an ndarray.
        
        Every bodyParts object, for now, is an iterable of bodyPart objects.
        The exact specification of such bodyPart object can be found in the ROS backend messages.
        For now, it contains at least a point in three dimensional space, with x, y and z coordinate.
        This method turns such an object into a numpy array that contains only the joints that are used acoording
        to our joint adapter.

        Args:
            body_parts: The body parts to be converted into a numpy array.
        
        Returns:
            An array corresponding to the used body parts found in the body_parts argument.
        """
        raise NotImplementedError("This is an interface, it should not be called directly.")

    def ndarray_to_body_parts(self, ndarray: np.ndarray) -> list:
        """Take an ndarray, and turn it into a Bodyparts object defined by the backend messages
        
        Every bodyParts object, for now, is an iterable of bodyPart objects.
        The exact specification of such bodyPart object can be found in the ROS backend messages.
        For now, it contains at least a point in three dimensional space, with x, y and z coordinate.
        This method turns such an object into a numpy array that contains only the joints that are used acoording
        to our joint adapter.

        Args:
            ndarray: An array representing parts parts.
        
        Returns:
            A Bodyparts object converted from the ndarray array.
        """
        raise NotImplementedError("Work in progress.")

    def recording_to_ndarray(self, recording: list) -> np.ndarray:
        """ Take a list of Bodyparts objects and turn them into an array of pose_arrays."""
        raise NotImplementedError("Work in progress.")


    
class SpinPoseDefinitionAdapter(PoseDefinitionAdapter):
    """This adapter uses the skeleton definition introduced to our system by Shawan Mohamed, originally formulated by the authors of the SPIN paper."""
    def __init__(self):
        # The indices of the joints that we use (of all the joints from the spin paper)
        self.joints_used = [0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 27, 28, 37, 39, 41, 42, 43];

        self.joint_labels = ['OP_Nose', 'OP_Neck', 'OP_R_Shoulder', 'OP_R_Elbow', 'OP_R_Wrist', 'OP_L_Shoulder', 'OP_L_Elbow', 'OP_L_Wrist', 'OP_Middle_Hip', 'OP_R_Hip', 'OP_R_Knee', 'OP_R_Ankle', 'OP_L_Hip', 'OP_L_Knee', 'OP_L_Ankle', 'OP_R_Eye', 'OP_L_Eye', 'OP_R_Ear', 'OP_L_Ear', 'OP_L_Big_Toe', 'OP_L_Small_Toe', 'OP_L_Heel', 'OP_R_Big_Toe', 'OP_R_Small_Toe', 'OP_R_Heel', 'R_Ankle', 'R_Knee', 'R_Hip', 'L_Hip', 'L_Knee', 'L_Ankle', 'R_Wrist', 'R_Elbow', 'R_Shoulder', 'L_Shoulder', 'L_Elbow', 'L_Wrist', 'Neck_LSP', 'Top_of_Head_LSP', 'Pelvis_MPII', 'Thorax_MPII', 'Spine_HM', 'Jaw_HM', 'Head_HM', 'Nose', 'L_Eye', 'R_Eye', 'L_Ear', 'R_Ear']

        self.joints_used_labels = [self.joint_labels[i] for i in self.joints_used]

        # Connections that connect two joints (in many cases bones)
        self.joint_connections = [[2, 3], [3, 4], [5, 6], [6, 7], [27, 9], [9, 12], [27, 28], [27, 10], [10, 11], [12, 13], [9, 10], [28, 12], [28, 13], [13, 14], [14, 21], [21, 20], [21, 19], [20, 19], [11, 24], [24, 22], [22, 23], [23, 24], [5, 28], [2, 27], [5, 2], [42, 17], [42, 18], [42, 0], [0, 15], [0, 16], [15, 16], [17, 43], [18, 43], [1, 37], [37, 43], [41, 37], [41, 39]]

        # The joint connections, represented with their lables from the spin paper
        # TODO: Maybe use sorted list for compatibility with C++
        self.joint_connections_labels = set(frozenset((self.joint_labels[x], self.joint_labels[y])) for [x, y] in self.joint_connections)

        self.center_of_body_label = 'Pelvis_MPII'

    def get_joint_index(self, joint_name: str):
        return self.joints_used_labels.index(joint_name)


    def recording_to_ndarray(self, recording: list) -> np.ndarray:
        array = np.ndarray(shape=[len(recording), len(self.joints_used), 3], dtype=np.float16)

        for idx_recording, step in enumerate(recording):
            skelleton = step[1]
            for joint, coordinates in skelleton.items():
                idx_step = self.get_joint_index(joint)
                array[idx_recording][idx_step][0] = coordinates['x']
                array[idx_recording][idx_step][1] = coordinates['y'] # We DO NOT have to swap x and y here, because Tamer has swapped it already (?)
                array[idx_recording][idx_step][2] = coordinates['z'] 
        
        return array


    def body_parts_to_ndarray(self, body_parts: Bodypart) -> np.ndarray:
        array = np.ndarray(shape=[len(self.joints_used), 3], dtype=np.float16)

        body_parts_used = [body_parts[i] for i in self.joints_used]

        for idx, bodyPart in enumerate(body_parts_used):
            array[idx][0] = bodyPart.point.x
            array[idx][1] = bodyPart.point.z
            array[idx][2] = bodyPart.point.y

        return array

    def ndarray_to_body_parts(self, ndarray: np.ndarray) -> list:
        # We need some dummy body parts that we do not actually use but are still part of the Person defined by SPIN
        body_parts = [Bodypart()] * len(self.joint_labels)

        for used_index, body_part_ndarray in zip(self.joints_used, ndarray):
            b = Bodypart()
            b.point.x = body_part_ndarray[0]
            b.point.y = body_part_ndarray[2]
            b.point.z = body_part_ndarray[1]
            body_parts[used_index] = b
        
        return body_parts


def discritize_feature_values(feature_value, last_resampled_feature_value, resolution):
    new_resampled_feature_values = []
    delta = feature_value - last_resampled_feature_value
    remaining_delta = abs(delta)
    
    while remaining_delta >= resolution:
        remaining_delta -= resolution
        last_resampled_feature_value += math.copysign(resolution, delta)
        new_resampled_feature_values.append(last_resampled_feature_value)

    return new_resampled_feature_values


def extract_average_hight_of_joints(pose_array: np.ndarray) -> float:
    """Compute the hight of a person as the average of the height of its joints.
    
    Args:
        pose: A pose, according to the ROS Message 'Person'.
        
    Returns:
        A float value corresponding to the hight of the person.
    """
    raise NotImplementedError("Work in progress.")
    
def extract_distance(pose_array: np.ndarray, joint_a: int, joint_b: int) -> float:
    """Compute the distance between two joints.

    Args:
        pose: A pose, according to the ROS Message 'Person'
        joint_a: Index of one of two joints between we measure the distance
        joint_b: Name of the the second joint to measure the distance

    Returns:
        A float value corresponding to the euclidean distance between joint_a and joint_b.
    """
    raise NotImplementedError("Work in progress.")

def extract_angle(pose_array: np.ndarray, inner_joint: int, outer_joints: set) -> float:
    """Compute the angle between three connected joints.
    
    Args:
        pose: A pose, according to the ROS Message 'Person'
        innter_joint: Index of the joint connected to both of the outer joints
        outer_joints: A set of two outer joint's indices

    Returns:
        A float value corresponding to the inner angle between the three defined joints.
    """
    def create_vector_from_two_points(a, b):
        return np.array([b[0] - a[0], b[1] - a[1], b[2] - a[2]])

    def dot_product(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    
    def length_of_vector(x):
        return math.sqrt(math.pow(x[0], 2) + math.pow(x[1], 2) + math.pow(x[2], 2))

    ba = create_vector_from_two_points(pose_array[inner_joint], pose_array[outer_joints[0]])
    bc = create_vector_from_two_points(pose_array[inner_joint], pose_array[outer_joints[1]])
    x = dot_product(ba, bc) / (length_of_vector(ba) * length_of_vector(bc))
    return math.acos(x) * 180 / math.pi


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
        last_values = [trajectory[0]]
        last_feature_state = decide_feature_state(last_values[-1], None, lower_boundary, upper_boundary)
        discrete_values = last_values
        feature_trajectory_indices = [0]
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
        discrete_trajectories_tensor.append(discrete_values)
        hankel_tensor.append(hankel(discrete_trajectories_tensor, np.roll(discrete_trajectories_tensor, -1)))
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
        median_resampled_values_reference_trajectory_fraction_from = np.average([discretization_reference_trajectory_indices_tensor[j, i]/recording_lengths[j] for j in range(len(recording_lengths))])
        median_resampled_values_reference_trajectory_fraction_to = np.average([discretization_reference_trajectory_indices_tensor[j, i + 1 % len(discretization_reference_trajectory_indices_tensor)]/recording_lengths[j] for j in range(len(recording_lengths))])
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
    decided_median_feature_states = median_feature_states_array[median_feature_states_array != 0]
    decided_median_feature_state_change_indices = np.where(decided_median_feature_states[:-1] * decided_median_feature_states[1:] < 0 )[0] + 1
    number_of_dicided_state_changes = len(decided_median_feature_state_change_indices)
    if decided_median_feature_states[0] != decided_median_feature_states[-1]:
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