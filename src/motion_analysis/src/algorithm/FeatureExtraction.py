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
from typing import Tuple

import hashlib
import numpy as np

try:
    from motion_analysis.src.algorithm.AlgoConfig import *
    from motion_analysis.src.algorithm.AlgoUtils import *
    from motion_analysis.src.algorithm.Features import *
    from motion_analysis.src.ROSAdapters import *
except (ModuleNotFoundError, ImportError):
    from src.algorithm.AlgoConfig import *
    from src.algorithm.AlgoUtils import *
    from src.algorithm.Features import *
    from src.ROSAdapters import *


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


def extract_average_height_of_joints(pose_array: np.ndarray) -> float:
    """Compute the height of a person as the average of the height of its joints.
    
    Args:
        pose_array: A pose, according to the ROS Message 'Person'.
        
    Returns:
        A float value corresponding to the height of the person.
    """
    raise NotImplementedError("see extract angle!")
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
    raise NotImplementedError("see extract angle!")

    return pose_array[index][Z]


def extract_height_of_body_core(pose_array: np.ndarray, pose_definition_adapter: PoseDefinitionAdapter) -> float:
    """Compute the average height of pelvis and neck, representing the height of the body's core.
    
    Args:
        pose_array: A pose, according to the ROS Message 'Person'.

    Returns:
    A float value corresponding to the height of the body's core.
    """
    raise NotImplementedError("see extract angle!")
    
    neck_idx = pose_definition_adapter.get_joint_index("Neck")
    pelvis_idx = pose_definition_adapter.get_joint_index("M_Hip")

    return np.average([pose_array[neck_idx][Z], pose_array[pelvis_idx][Z]])
    

def extract_distance(pose_array: np.ndarray, specification_dict, pose_definition_adapter) -> float:
    """Compute the distance between two joints.

    Args:
        pose: A pose, according to the ROS Message 'Person'
        joint_a: Index of one of two joints between we measure the distance
        joint_b: Name of the the second joint to measure the distance

    Returns:
        A float value corresponding to the euclidean distance between joint_a and joint_b.
    """
    joint_a = specification_dict['joints'][0]
    joint_b = specification_dict['joints'][1]
    joint_a_idx = pose_definition_adapter.get_joint_index(joint_a)
    joint_b_idx = pose_definition_adapter.get_joint_index(joint_b)
    vector = create_vector_from_two_points(pose_array[joint_a_idx], pose_array[joint_b_idx])

    return length_of_vector(vector)


def extract_angle(pose_array: np.ndarray, specification_dict, pose_definition_adapter) -> float:
    """Compute the angle between three connected joints.
    
    Args:
        pose: A pose, according to the ROS Message 'Person'
        innter_joint: Index of the joint connected to both of the outer joints
        outer_joints: A set of two outer joint's indices

    Returns:
        A float value corresponding to the inner angle between the three defined joints.
    """

    inner_joint_name, outer_joints_names = specification_dict["inner_joint"], specification_dict["outer_joints"]
    inner_joint_idx = pose_definition_adapter.get_joint_index(inner_joint_name)
    outer_joint_idxs = tuple(pose_definition_adapter.get_joint_index(n) for n in outer_joints_names)

    ba = create_vector_from_two_points(pose_array[inner_joint_idx], pose_array[outer_joint_idxs[0]])
    bc = create_vector_from_two_points(pose_array[inner_joint_idx], pose_array[outer_joint_idxs[1]])
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


def extract_feature_of_interest_specification_dictionary(hmi_features: dict, pose_definition_adapter: PoseDefinitionAdapter) -> dict:
    """This method turns an exercise data dictionary into a dictionary of features.

    Features are specified according to the following structure:
    {
        '<feature_hash>': <feature_value>,
        (...)
    }
    """

    features_of_interest = dict()
    for f in hmi_features:
        if f['type'] == "angle":
            features_of_interest.update(extract_angles_of_interest(f['value'], pose_definition_adapter))
        elif f['type'] == "distance":
            feature_strings = f['value']
            feature_hash = hashlib.md5(sorted(feature_strings).__repr__().encode()).digest()
            features_of_interest.update({feature_hash: {"type": FeatureType.JOINT_DISTANCE, "joints": f['value']}})
        else:
            log("Unhandled feature type:" + str(f['type']))

    return features_of_interest


def extract_angles_of_interest(joint_names: list, pose_definition_adapter: PoseDefinitionAdapter) -> dict:
    frozen_joint_names = frozenset(joint_names)
    
    exceptions = dict()
    features_of_interest = {}

    inner_joint = pose_definition_adapter.find_inner_joint(joint_names)
    outer_joints = set(joint_names)
    
    outer_joints.remove(inner_joint)

    
    if exceptions:
        rp.logerr("Errors occured while parsing the provided exercise:" + str(exceptions))
    
    joint_hash = hashlib.md5(sorted(frozen_joint_names).__repr__().encode()).digest()
    features_of_interest[joint_hash] = {"type": FeatureType.ANGLE, "inner_joint": inner_joint, "outer_joints": outer_joints}

    return features_of_interest


feature_extraction_methods = {FeatureType.ANGLE: extract_angle, FeatureType.JOINT_DISTANCE: extract_distance}