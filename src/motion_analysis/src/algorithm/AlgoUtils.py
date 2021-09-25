#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains a code snippets that have nowhere else to go.
"""

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
    return complex(np.sin(progress*2*np.pi), np.cos(progress*2*np.pi))


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
                widget.update_reference_plots.emit(
                    np.array(sample_reference_feature.values), \
                    np.array(sample_reference_feature.filtered_values), \
                    np.array(sample_reference_feature.discretized_values))
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
    gui.update_overall_data_signal.emit(int(progress), int(alignment), np.array([progress_alignment_vector.real, progress_alignment_vector.imag]))
    
