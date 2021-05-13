"""
This file contains code that deals with the  manual extraction of low-level features from skelletons.
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
    ],
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
from src.config import *
from src.Util import *

from scipy.linalg import hankel
from scipy.spatial.distance import euclidean
from backend.msg import Bodypart


class FeatureExtractorException(Exception):
    pass

class UnknownAngleException(FeatureExtractorException):
    pass


class FeatureExtractor():
    """This class takes ndarrays reporesenting human poses and turns them into features."""
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
        raise NotImplementedError("This is an interface, it should not be called directly.")

    def recording_to_ndarray(self, recording: list) -> np.ndarray:
        """ Take a list of Bodyparts objects and turn them into an array of pose_arrays."""
        raise NotImplementedError("This is an interface, it should not be called directly.")
        
    def extract_distance(self, pose_array: np.ndarray, joint_a: int, joint_b: int) -> float:
        """Compute the distance between two joints.

        Args:
            pose: A pose, according to the ROS Message 'Person'
            joint_a: Index of one of two joints between we measure the distance
            joint_b: Name of the the second joint to measure the distance

        Returns:
            A float value corresponding to the euclidean distance between joint_a and joint_b.
        """
        raise NotImplementedError("Currently not part of the features of interest.")

    def extract_angle(self, pose_array: np.ndarray, inner_joint: int, outer_joints: set) -> float:
        """Compute the angle between three connected joints.
        
        Args:
            pose: A pose, according to the ROS Message 'Person'
            innter_joint: Index of the joint connected to both of the outer joints
            outer_joints: A set of two outer joint's indices

        Returns:
            A float value corresponding to the inner angle between the three defined joints.
        """
        # TODO: Extract angles for trajectories of poses
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

    @abstractmethod
    def extract_rotation(self, pose_array: np.ndarray, joint: int) -> float:
        """Compute the rotation of a joint.

       Args:
            pose: A pose, according to the ROS Message 'Person'
            joint: Index of the joint

        Returns:
            TODO: Specifiy our understanding of rotation (relative to what?)
        """

        raise NotImplementedError("Currently not part of the features of interest.")

    @abstractmethod
    def extract_speed(self, poses_array: np.ndarray, joint: int) -> float:
        """Compute the speed of a joint, averaged over a list of poses.

        Args:
            poses: A list of poses, according to the ROS Message 'Person'
            joint: Index of the joint

        Returns:
            A float value corresponding to the speed of the joint, averaged over the list of poses.
        """
        raise NotImplementedError("Currently not part of the features of interest.")

    @abstractmethod
    def extract_acceleration(self, poses_array: np.ndarray, joint: int) -> float:
        """Compute the acceleration of a joint, averaged over a list of poses.

        Args:
            poses: A list of poses, according to the ROS Message 'Person'
            joint: Index of the joint

        Returns:
            A float value corresponding to the acceleration of the joint, averaged over the list of poses. 
        """
        raise NotImplementedError("Currently not part of the features of interest.")

    def extract_feature_of_interest_specification_dictionary(self, exercise_data: dict) -> dict:
        """This method turns an exercise data dictionary into a dictionary of features.

        Features are specified according to the following structure:
        { # one entry per feature type
            'angles': <list of angles>,
            (...)
        }
        """
        features_of_interest = dict()
        for feature_name, extraction_method in [('angles', self.extract_angles_of_interest)]:
            features_of_interest[feature_name] = extraction_method(exercise_data)

        return features_of_interest

    def extract_angles_of_interest(self, exercise_data: dict) -> dict:
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
            """Take 3 joints and look their connections up in self.joint_connection_labels to see if there is an inner joint.
            
            Args:
                joint_names: 
            """
            if not len(joint_names) == 3:
                raise FeatureExtractorException("Can not calculate angle between other than 3 points.")
            
            joint_connection_combinations = combinations(joint_names, 2)

            # We must find two connections between our three joints, such that there is a usefull angle between them
            combinations_found = 0

            for joint_connection_combination in joint_connection_combinations:
                if frozenset(joint_connection_combination) not in self.joint_connections_labels:
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

    def extract_states(self, pose_array: np.ndarray, boundaries: dict, feature_of_interest_specification: dict):
        """Extract the state.
        
        In order to so, we take a naive approach and test how many of the angles of interest in the first step of the recording tend towards to higher boundary.
        """
        state_dict = dict()

        for feature_type, feature_specification in feature_of_interest_specification.items():
            state_dict[feature_type] = dict()
            if feature_type == 'angles':
                # Subroutine that calculates the boundaries for angles
                for joint_hash, features in feature_specification.items():
                    state_dict[feature_type][joint_hash] = dict()
                    inner_joint_name, outer_joints_names = features["inner_joint"], features["outer_joints"]
                    inner_joint_idx = self.get_joint_index(inner_joint_name)
                    outer_joint_idxs = tuple(self.get_joint_index(n) for n in outer_joints_names)
 
                    angle = self.extract_angle(pose_array, inner_joint_idx, outer_joint_idxs)

                    # TODO: From here on, reuse code in this loop for other feature types

                    lower_angle = boundaries["angles"][joint_hash]["lower_boundary"]
                    higher_angle = boundaries["angles"][joint_hash]["upper_boundary"]

                    feature_dict = {"feature_value": angle}

                    if angle > higher_angle:
                        feature_dict["feature_state"] = FEATURE_HIGH
                    elif angle < lower_angle:
                        feature_dict["feature_state"] = FEATURE_LOW
                    else:
                        # TODO: possibly leave out middle state and just return incopmlete dict without middle states
                        feature_dict["feature_state"] = FEATURE_UNDECIDED

                    state_dict[feature_type][joint_hash] = feature_dict
            else:
                raise NotImplementedError("Trying to extract states for an unspecified feature type")
        
        return state_dict
    
    def extract_feature_trajectories_from_recordings(self, recordings: List[np.ndarray], feature_of_interest_specification: dict) -> dict:
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
        
        Returns:
            The boundary specification, which follows the same structure as the features_of_interest_specification.
        """
        feature_trajectory_dict = dict()
        
        for feature_type, feature_specification in feature_of_interest_specification.items():
            feature_trajectory_dict[feature_type] = dict()
            if feature_type == 'angles':
                # Subroutine that calculates the boundaries for angles
                for joint_hash, features in feature_specification.items():
                    feature_trajectory_dict[feature_type][joint_hash] = dict()
                    inner_joint_name, outer_joints_names = features["inner_joint"], features["outer_joints"]
                    inner_joint_idx = self.get_joint_index(inner_joint_name)
                    outer_joint_idxs = tuple(self.get_joint_index(n) for n in outer_joints_names)
                    recording_angles_list = []
                    for recording in recordings:
                        recording_angles = []
                        for pose_array in recording:
                            recording_angles.append(self.extract_angle(pose_array, inner_joint_idx, outer_joint_idxs))
                        recording_angles_list.append(recording_angles)

                    feature_trajectory_dict[feature_type][joint_hash] = recording_angles_list
            else:
                raise NotImplementedError("Trying to extract boundaries for an unspecified feature type")

        return feature_trajectory_dict

    def extract_reference_feature_data_from_feature_trajectories(self, feature_trajectories: dict) -> dict:
        """Extract the boundaries of angles (and in the future possibly distances).
            
        This method takes a dict consisting of feature trajectories and turns these trajectories into boudnaries
        that take the places of the trajectories in the dict.

        Args:
            feature_trajectories: A nested dictionary which looks as follows:
            { # One entry for each feature type
                <feature type>: <list of feature lists>,
                (...)
            }
        
        Returns:
            The boundary specification, which follows the same structure as the features of interest specification.
        """

        def extract_reference_feature_data_of_child_dictionary(d): #TODO: Rename me
            new_d = {}
            for k, feature_trajectories in d.items():
                if isinstance(feature_trajectories, collections.MutableMapping):
                    new_d[k] = extract_reference_feature_data_of_child_dictionary(feature_trajectories)
                else:
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

                    # We compute the resampled values separately for each trajectory and turn them into hankel matrices
                    resolution = range_of_motion * FEATURE_TRAJECTORY_RESOLUTION_FACTOR
                    resampled_trajectories = []
                    resampled_values_reference_trajectory_indices = [] # For every resampled value, we need an index to point us to the original pose
                    reference_trajectory_hankel_matrices = [] # In the comparing algorithm, we need a hankel matrix of every resampled reference trajectory
                    for trajectory in feature_trajectories:
                        last_values = [trajectory[0]]
                        resampled_values = last_values
                        feature_trajectory_indices = [0]
                        for index, value in enumerate(trajectory):
                            value_turned_into_resampled_values = compute_resampled_feature_values(value, last_values[-1], resolution)
                            resampled_values.extend(value_turned_into_resampled_values)
                            if value_turned_into_resampled_values:
                                last_values = value_turned_into_resampled_values
                                feature_trajectory_indices.extend([index] * len(last_values))
                        resampled_trajectories.append(resampled_values)
                        reference_trajectory_hankel_matrices.append(hankel(resampled_trajectories, np.roll(resampled_trajectories, -1)))
                        resampled_values_reference_trajectory_indices.append(feature_trajectory_indices)

                    reference_trajectory_hankel_matrices = np.asarray(reference_trajectory_hankel_matrices, dtype=np.float16)
                    # resampled_values_arrays = np.asarray(resampled_v, dtype=np.float16)

                    resampled_reference_trajectory_scale = set()
                    for resampled_trajectory in resampled_trajectories:
                        resampled_reference_trajectory_scale.update(set(resampled_trajectory))

                    resampled_reference_trajectory_scale_array = np.sort(list(resampled_reference_trajectory_scale))
                        
                    new_d[k] = {"lower_boundary": lower_boundary, "upper_boundary": upper_boundary, "lowest_value": lowest_value, "highest_value": highest_value, "range_of_motion": range_of_motion, "resampled_values_reference_trajectory_indices": resampled_values_reference_trajectory_indices, "reference_trajectory_hankel_matrices": reference_trajectory_hankel_matrices, "resampled_reference_trajectory_scale_array": resampled_reference_trajectory_scale_array}

            return new_d
        
        return extract_reference_feature_data_of_child_dictionary(feature_trajectories)

    
class SpinFeatureExtractor(FeatureExtractor):
    """ This FeatureExtractor is able to extract features from ndarrays that are formatted according to the SPIN paper."""
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
        """Get the index of a joint from its name"""
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


def compute_resampled_feature_values(feature_value, last_resampled_feature_value, resolution):
    new_resampled_feature_values = []
    delta = feature_value - last_resampled_feature_value
    remaining_delta = abs(delta)
    
    while remaining_delta >= resolution:
        remaining_delta -= resolution
        # Timestamp the resampled values, so that we do not add them multiple times to the queue later
        last_resampled_feature_value += math.copysign(resolution, delta)
        new_resampled_feature_values.append(last_resampled_feature_value)

    return new_resampled_feature_values
