#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Skelleton Utility.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
Based on Ted's https://github.com/Basti110/skeleton-normalization/blob/main/convert.py
"""

from typing import Set
import copy
import numpy as np
import rospy as rp
from sklearn.preprocessing import normalize

# This is me, Artur, doing military press
# All skelletons are normalized to my skelleton. :)
normal_skelleton = np.load('/home/trainerai/trainerai-core/src/motion_analysis/standard_skelleton.npy') #, allow_pickle=True)

# Build order of the current TrainerAi skeleton
BODY_BUILD_ORDER = [(0, 3), (3, 6), (6, 9), (9, 12), (12, 15), (12, 13), (13, 16), (16, 18), (18, 20), (20, 22), (12, 14), (14, 17),
(17, 19), (19, 21), (21, 23), (0, 1), (1, 4), (4, 7), (7, 10), (0, 2), (2, 5), (5, 8), (8, 11)]

CENTRAL_JOINT_IDX = 0 # Choose this to be something like the pelvis and a joint in the back, skelletons are rotatet sich that this bone always overlaps
# We do not know the orientation of the ground, so we need to orient skelletons along each other and not to the ground
ORIENTATION_VECTOR_JOINT_IDXS_1 = (0, 1)
ORIENTATION_VECTOR_JOINT_IDXS_2 = (0, 2)


def calculate_orientation_vector(input_skelleton):
    """Returns the orientation of the skelleton's pelvis.

    Calculateas the orientation of the pelvis joint connections of the skelleton.
    The returned orientation is the vector representing the normal vector of two of the pelvises' joint connections.
    """
    # Roate skelleton, such that skelletons pelvises overlap
    x = np.array(input_skelleton[ORIENTATION_VECTOR_JOINT_IDXS_1[1]] - input_skelleton[ORIENTATION_VECTOR_JOINT_IDXS_1[0]])
    y = np.array(input_skelleton[ORIENTATION_VECTOR_JOINT_IDXS_2[1]] - input_skelleton[ORIENTATION_VECTOR_JOINT_IDXS_2[0]])
    assert not np.array_equal(x, y)
    _x = x / np.linalg.norm(x)
    cross_x_y = np.cross(x,y)
    _z = cross_x_y / np.linalg.norm(cross_x_y)
    cross__z_x = np.cross(_z,x)
    _y = cross__z_x / np.linalg.norm(cross__z_x)
    return  np.stack([_x, _y, _z])

normal_orientation_matrix_transpose = np.matrix.transpose(calculate_orientation_vector(normal_skelleton))


def resize_len_vec1_to_vec2(vec1, vec2):
    length_vec1 = np.linalg.norm(vec1)
    length_vec2 = np.linalg.norm(vec2)
    return (length_vec2 / length_vec1) * vec1


def normalize_skelleton_size(input_skelleton):
    """Stretch all vectors of person to the same length as the vectors in normal_skelleton

    BODY_BUILD_ORDER: The order to build a completly new skeleton with the old vectors. The first Index in the first tuple
    is the start point. It is the only point with the same coordinates as before. For the new skeleton, the algorithm
    takes the vector direction of person and the vecttor length of person2.
    """
    resized_skelleton = np.copy(input_skelleton)

    first_joint_idx = BODY_BUILD_ORDER[0][0]
    resized_skelleton[first_joint_idx] = np.copy(input_skelleton[first_joint_idx])

    for joint_idxs in BODY_BUILD_ORDER:
        joint1_p1 = input_skelleton[joint_idxs[0]]
        joint2_p1 = input_skelleton[joint_idxs[1]]

        joint1_p2 = normal_skelleton[joint_idxs[0]]
        joint2_p2 = normal_skelleton[joint_idxs[1]]
        point1_p2 = np.array([joint1_p2[0], joint1_p2[1], joint1_p2[2]])
        point2_p2 = np.array([joint2_p2[0], joint2_p2[1], joint2_p2[2]])
        vec_joints_person2 = np.array(point2_p2 - point1_p2)
        
        point1_p1 = np.array([joint1_p1[0], joint1_p1[1], joint1_p1[2]])
        point2_p1 = np.array([joint2_p1[0], joint2_p1[1], joint2_p1[2]])
        vec_joints_person1 = np.array(point2_p1 - point1_p1)

        new_vec_p1 = resize_len_vec1_to_vec2(vec_joints_person1, vec_joints_person2)

        base_joint = resized_skelleton[joint_idxs[0]]
        resized_skelleton[joint_idxs[1]] = np.array([base_joint[0] + new_vec_p1[0], base_joint[1] + new_vec_p1[1], base_joint[2] + new_vec_p1[2]]) 

    return resized_skelleton


def normalize_skelleton_position(input_skelleton):
    """Returns a skelleton with the pelvis at (0, 0, 0)."""
    reoriented_skelleton = np.copy(input_skelleton)
    # Move skelleton to 0/0/0 by substracting coordinates of central joint from all joints
    for joint_idx in range(len(reoriented_skelleton)):
        reoriented_skelleton[joint_idx][0] = input_skelleton[joint_idx][0] - input_skelleton[CENTRAL_JOINT_IDX][0]
        reoriented_skelleton[joint_idx][1] = input_skelleton[joint_idx][1] - input_skelleton[CENTRAL_JOINT_IDX][1]
        reoriented_skelleton[joint_idx][2] = input_skelleton[joint_idx][2] - input_skelleton[CENTRAL_JOINT_IDX][2]

    return reoriented_skelleton

def normalize_skelleton_orientation(input_skelleton):
    """Returns a skelleton with the pevlis turned into the same directions as the normal skelleton."""
    reoriented_skelleton = np.copy(input_skelleton)
    reoriented_skelleton_rotation_matrix = calculate_orientation_vector(reoriented_skelleton)
    rotation_matrix_to_normal_orientation = np.matmul(normal_orientation_matrix_transpose, reoriented_skelleton_rotation_matrix)

    for joint_idx in range(len(reoriented_skelleton)):
        reoriented_skelleton[joint_idx] = np.matmul(reoriented_skelleton[joint_idx], rotation_matrix_to_normal_orientation)

    return reoriented_skelleton

def skelleton_coherency_test():
    """Check whether all joints are connected."""
    already_added_nodes : Set[int] = set({})
    already_added_nodes.add(BODY_BUILD_ORDER[0][0])
    for joint in BODY_BUILD_ORDER:
        if joint[0] not in already_added_nodes:
            rp.logerr(f"ERROR: Index {joint[0]} (first index) was not added in the Set before")
        if joint[1] in already_added_nodes:
            rp.logerr(f"ERROR: Index {joint[0]} (second index) was added in the Set before")
        already_added_nodes.add(joint[1])
    rp.logerr(f"Total number of joints: {len(already_added_nodes)}")
    rp.logerr(already_added_nodes)

def normalize_skelleton(input_skelleton):
    """Utility method to normalize size, position and orientation of skelleton."""
    skelleton = normalize_skelleton_size(input_skelleton)
    skelleton = normalize_skelleton_position(skelleton)
    skelleton = normalize_skelleton_orientation(skelleton)
    return skelleton