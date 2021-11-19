#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains a code snippets that have nowhere else to go.
"""

try:
    from motion_analysis.src.algorithm.SkelletonUtility import *
except ImportError:
    from src.algorithm.SkelletonUtility import *

from typing import Any
import numpy as np
from abc import abstractmethod

class PoseDefinitionAdapter():
    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def get_joint_index(self, joint_name: str) -> int:
        """Get the index of a joint from its name"""
        raise NotImplementedError("This is an interface, it should not be called directly.")

    @abstractmethod
    def poses_to_ndarray(self, poses: object) -> np.ndarray:
        """Take a poses object, defined by the backend messages turn it into an ndarray.
        
        Every poses object, for now, is an iterable of bodyPart objects.
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

    def ndarray_to_poses(self, ndarray: np.ndarray) -> list:
        """Take an ndarray, and turn it into a Bodyparts object defined by the backend messages
        
        Every poses object, for now, is an iterable of pose objects.
        The exact specification of such bodyPart object can be found in the ROS backend messages.
        For now, it contains at least a point in three dimensional space, with x, y and z coordinate.
        This method turns such an object into a numpy array that contains only the joints that are used acoording
        to our joint adapter.

        Args:
            ndarray: An array representing parts parts.
        
        Returns:
            A poses object converted from the ndarray array.
        """
        raise NotImplementedError("Work in progress.")

    def recording_to_ndarray(self, recording: list) -> np.ndarray:
        """ Take a list of Bodyparts objects and turn them into an array of pose_arrays."""
        raise NotImplementedError("Work in progress.")

    def normalize_skelletons(self, recording: np.ndarray) -> np.ndarray:
        new_recording = list()
        for pose_array in recording:
            new_recording.append(normalize_skelleton(pose_array))
        
        return np.array(new_recording)



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
    progress = np.angle(progress_vector_sum) + (2 * np.pi)
    progress = progress % (2 * np.pi) # Bring progress back our notation of a value in range (0...1)
    progress = progress / (2 * np.pi)
    if vectors:
        alignment = np.abs(progress_vector_sum) / max(1, len(vectors))
        progress_alignment_vector = progress_vector_sum / max(1, len(vectors))
    else:
        alignment = 0
        progress_alignment_vector = progress_vector_sum
    
    return  progress, alignment, progress_alignment_vector


def fast_hash(o: Any) -> str:
    """ Hashes object with their string representation, but numpy arrays as a whole."""
    if issubclass(type(o), np.ndarray):
        # TODO: Check if this is efficient
        return hash(o.tobytes())
    return hash(str(o))


def custom_metric(a, b):
    """ Calculate the absolute difference between two ranges on a "ring" scale between 0 and 1."""
    a_from = a["median_resampled_values_reference_trajectory_fraction_from"]
    a_to = a["median_resampled_values_reference_trajectory_fraction_to"]
    b_from = b["median_resampled_values_reference_trajectory_fraction_from"]
    b_to = b["median_resampled_values_reference_trajectory_fraction_to"]

    if b_from <= a_from <= b_to or b_from <= a_to <= b_to:
        return 0
    else:
        return min([abs(a_from - b_to), abs(b_from - a_to), abs(a_from + 1 - b_to), abs(b_from + 1 - a_to)])


def update_gui_features(gui, feature):
    """ Update a gui regarding a feature if applicable. """
    if not gui:
        return
    feature_hash = feature.feature_hash
    if feature_hash in gui.feature_widgets.keys():
        widget = gui.feature_widgets[feature_hash]
        if not widget.reference_plot_data_set:
            try:
                sample_reference_feature = feature.reference_feature_collection.reference_recording_features[0]
                spec = feature.specification_dict
                spec.pop("type")
                widget.update_static_data.emit(
                    np.array(sample_reference_feature.values), \
                    np.array(sample_reference_feature.filtered_values), \
                    np.array(sample_reference_feature.discretized_values), \
                    str(feature.type), \
                    str(spec))
            except KeyError:
                pass
        
            
        widget.update_user_data.emit(np.array(feature.values), \
            np.array(feature.filtered_values), \
                np.array(feature.discretized_values), \
                    np.array(feature.errors), \
                        np.array([feature.progress_vector.real, feature.progress_vector.imag]), \
                            np.array(feature.prediction))


def update_gui_progress(gui, progress, alignment, progress_alignment_vector):
    """ Update a gui regarding overall parameters. """
    if not gui:
        return
    gui.update_overall_data_signal.emit(float(progress), float(alignment), np.array([progress_alignment_vector.real, progress_alignment_vector.imag]))
    
    
# from src/AI/spin/utils/geometry.py
# def quat_to_rotmat(quat):
#     """Convert quaternion coefficients to rotation matrix.
#     Args:
#         quat: size = [B, 4] 4 <===>(w, x, y, z)
#     Returns:
#         Rotation matrix corresponding to the quaternion -- size = [B, 3, 3]
#     """ 
#     norm_quat = quat
#     norm_quat = norm_quat/norm_quat.norm(p=2, dim=1, keepdim=True)
#     w, x, y, z = norm_quat[:,0], norm_quat[:,1], norm_quat[:,2], norm_quat[:,3]

#     B = quat.size(0)

#     w2, x2, y2, z2 = w.pow(2), x.pow(2), y.pow(2), z.pow(2)
#     wx, wy, wz = w*x, w*y, w*z
#     xy, xz, yz = x*y, x*z, y*z

#     rotMat = torch.stack([w2 + x2 - y2 - z2, 2*xy - 2*wz, 2*wy + 2*xz,
#                           2*wz + 2*xy, w2 - x2 + y2 - z2, 2*yz - 2*wx,
#                           2*xz - 2*wy, 2*wx + 2*yz, w2 - x2 - y2 + z2], dim=1).view(B, 3, 3)
#     return rotMat    


# from src/AI/spin/utils/imutils.py
# def get_transform(center, scale, res, rot=0):
#     """Generate transformation matrix."""
#     h = 200 * scale
#     t = np.zeros((3, 3))
#     t[0, 0] = float(res[1]) / h
#     t[1, 1] = float(res[0]) / h
#     t[0, 2] = res[1] * (-float(center[0]) / h + .5)
#     t[1, 2] = res[0] * (-float(center[1]) / h + .5)
#     t[2, 2] = 1
#     if not rot == 0:
#         rot = -rot # To match direction of rotation from cropping
#         rot_mat = np.zeros((3,3))
#         rot_rad = rot * np.pi / 180
#         sn,cs = np.sin(rot_rad), np.cos(rot_rad)
#         rot_mat[0,:2] = [cs, -sn]
#         rot_mat[1,:2] = [sn, cs]
#         rot_mat[2,2] = 1
#         # Need to rotate around center
#         t_mat = np.eye(3)
#         t_mat[0,2] = -res[1]/2
#         t_mat[1,2] = -res[0]/2
#         t_inv = t_mat.copy()
#         t_inv[:2,2] *= -1
#         t = np.dot(t_inv,np.dot(rot_mat,np.dot(t_mat,t)))
#     return t

# def transform(pt, center, scale, res, invert=0, rot=0):
#     """Transform pixel location to different reference."""
#     t = get_transform(center, scale, res, rot=rot)
#     if invert:
#         t = np.linalg.inv(t)
#     new_pt = np.array([pt[0]-1, pt[1]-1, 1.]).T
#     new_pt = np.dot(t, new_pt)
#     return new_pt[:2].astype(int)+1