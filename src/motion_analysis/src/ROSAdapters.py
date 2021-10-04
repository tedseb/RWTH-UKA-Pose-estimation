#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains code that deals with adapting code that is based on numpy to ROS messages.
"""

from abc import abstractmethod

import numpy as np

try:
    from motion_analysis.src.DataConfig import *
    from motion_analysis.src.DataUtils import *
    from motion_analysis.src.algorithm.AlgoConfig import X, Y, Z
    from motion_analysis.src.algorithm.AlgoUtils import PoseDefinitionAdapter
except ImportError:
    from src.DataConfig import *
    from src.DataUtils import *
    from src.algorithm.AlgoConfig import X, Y, Z
    from src.algorithm.AlgoUtils import PoseDefinitionAdapter

try:
    from backend.msg import Bodypart
    from visualization_msgs.msg import Marker, MarkerArray
except ModuleNotFoundError:
    # We are not running python from the ROS venv
    Bodypart = Marker = MarkerArray = object


class FeatureExtractorException(Exception):
    pass

class UnknownAngleException(FeatureExtractorException):
    pass


class MetrabsPoseDefinitionAdapter(PoseDefinitionAdapter):
    """This adapter uses the skeleton definition introduced to our system by Shawan Mohamed, originally formulated by the authors of the SPIN paper."""
    def __init__(self):
        # The indices of the joints that we use (of all the joints from the spin paper)
        self.joints_used = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23]

        self.joint_labels = ['M_Hip', 'L_Hip', 'R_Hip', 'L_Back', 'L_Knee', 'R_Knee', 'M_Back', 'L_Ankle', 'R_Ankle', 'U_Back', 'L_Toes', 'R_Toes', 'Neck', 'L_Collarbone', 'R_Collarbone', 'Head', 'L_Shoulder', 'R_Shoulder', 'L_Elbow', 'R_Elbow', 'L_Wrist', 'R_Wrist', 'L_Fingers', 'R_Fingers']

        self.joints_used_labels = [self.joint_labels[i] for i in self.joints_used]

        # Connections that connect two joints (in many cases bones)
        self.joint_connections = [[1,4], [1,0], [2,5], [2,0], [3,6], [3,0], [4,7], [5,8], [6,9], [7,10], [8,11], [9,12], [12,13], [12,14], [12,15], [13,16], [14,17], [16,18], [17,19], [18,20], [19,21], [20,22], [21,23]]

        # The joint connections, represented with their lables from the spin paper
        # TODO: Maybe use sorted list for compatibility with C++
        self.joint_connections_labels = set(frozenset((self.joint_labels[x], self.joint_labels[y])) for [x, y] in self.joint_connections)

        self.center_of_body_label = 'M_Hip'

    def get_joint_index(self, joint_name: str):
        return self.joints_used_labels.index(joint_name)


    def recording_to_ndarray(self, recording: list) -> np.ndarray:
        array = np.ndarray(shape=[len(recording), len(self.joints_used), 3], dtype=np.float16)

        for idx_recording, step in enumerate(recording):
            skelleton = step[1]
            for joint, coordinates in skelleton.items():
                idx_step = self.get_joint_index(joint)
                array[idx_recording][idx_step][X] = coordinates['x']
                array[idx_recording][idx_step][Y] = coordinates['y'] # We DO NOT have to swap x and y here, because Tamer has swapped it already (?)
                array[idx_recording][idx_step][Z] = coordinates['z'] 
        
        return array


    def pose_to_nd_array(self, pose: dict):
        array = np.ndarray(shape=[len(self.joints_used), 3], dtype=np.float16)
        for joint, coordinates in pose.items():
                idx_step = self.get_joint_index(joint)
                array[idx_step][X] = coordinates['x']
                array[idx_step][Y] = coordinates['y'] # We DO NOT have to swap x and y here, because Tamer has swapped it already (?)
                array[idx_step][Z] = coordinates['z'] 
        return array


    def body_parts_to_ndarray(self, body_parts: Bodypart) -> np.ndarray:
        array = np.ndarray(shape=[len(self.joints_used), 3], dtype=np.float16)

        body_parts_used = [body_parts[i] for i in self.joints_used]

        for idx, bodyPart in enumerate(body_parts_used):
            array[idx][X] = bodyPart.point.x
            array[idx][Y] = bodyPart.point.z
            array[idx][Z] = bodyPart.point.y

        return array

    def ndarray_to_body_parts(self, ndarray: np.ndarray) -> list:
        # We need some dummy body parts that we do not actually use but are still part of the Person defined by SPIN
        body_parts = [Bodypart()] * len(self.joint_labels)

        for used_index, body_part_ndarray in zip(self.joints_used, ndarray):
            b = Bodypart()
            b.point.x = body_part_ndarray[X]
            b.point.y = body_part_ndarray[Z]
            b.point.z = body_part_ndarray[Y]
            body_parts[used_index] = b
        
        return body_parts



    """
    This adapter uses the skeleton definition introduced to our system by Shawan Mohamed, originally formulated by the authors of the SPIN paper.
    The Adapter is considered legacy, because we use the Metrabs code now by default.
    """
    def __init__(self):
        # The indices of the joints that we use (of all the joints from the spin paper)
        self.joints_used = [0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 27, 28, 37, 39, 41, 42, 43]

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
                array[idx_recording][idx_step][X] = coordinates['x']
                array[idx_recording][idx_step][Y] = coordinates['y'] # We DO NOT have to swap x and y here, because Tamer has swapped it already (?)
                array[idx_recording][idx_step][Z] = coordinates['z'] 
        
        return array


    def pose_to_nd_array(self, pose: dict):
        array = np.ndarray(shape=[len(self.joints_used), 3], dtype=np.float16)
        for joint, coordinates in pose.items():
                idx_step = self.get_joint_index(joint)
                array[idx_step][X] = coordinates['x']
                array[idx_step][Y] = coordinates['y'] # We DO NOT have to swap x and y here, because Tamer has swapped it already (?)
                array[idx_step][Z] = coordinates['z'] 
        return array


    def body_parts_to_ndarray(self, body_parts: Bodypart) -> np.ndarray:
        array = np.ndarray(shape=[len(self.joints_used), 3], dtype=np.float16)

        body_parts_used = [body_parts[i] for i in self.joints_used]

        for idx, bodyPart in enumerate(body_parts_used):
            array[idx][X] = bodyPart.point.x
            array[idx][Y] = bodyPart.point.z
            array[idx][Z] = bodyPart.point.y

        return array

    def ndarray_to_body_parts(self, ndarray: np.ndarray) -> list:
        # We need some dummy body parts that we do not actually use but are still part of the Person defined by SPIN
        body_parts = [Bodypart()] * len(self.joint_labels)

        for used_index, body_part_ndarray in zip(self.joints_used, ndarray):
            b = Bodypart()
            b.point.x = body_part_ndarray[X]
            b.point.y = body_part_ndarray[Z]
            b.point.z = body_part_ndarray[Y]
            body_parts[used_index] = b
        
        return body_parts

