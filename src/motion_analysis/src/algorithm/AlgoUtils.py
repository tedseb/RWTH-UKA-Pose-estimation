#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains a code snippets that have nowhere else to go.
"""


from typing import Any
import numpy as np
from abc import abstractmethod


class IllegalAngleException(Exception):
    pass

class PoseDefinitionAdapter():
    @abstractmethod
    def __init__(self):
        self.normal_orientation_matrix_transpose = np.matrix.transpose(self.calculate_orientation_vector(self.normal_skelleton))

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
            new_recording.append(self.normalize_skelleton(pose_array))
        
        return np.array(new_recording)

    def get_joint_index(self, joint_name: str):
        return self.joints_used_labels.index(joint_name)

    def get_joint_name(self, joint_idx: int):
        return self.joints_used_labels[joint_idx]

    def calculate_orientation_vector(self, input_skelleton):
        """Returns the orientation of the skelleton's pelvis.

        Calculateas the orientation of the pelvis joint connections of the skelleton.
        The returned orientation is the vector representing the normal vector of two of the pelvises' joint connections.
        """
        # Roate skelleton, such that skelletons pelvises overlap
        x = np.array(input_skelleton[self.orientational_vector_joint_idxs_1[1]] - input_skelleton[self.orientational_vector_joint_idxs_1[0]])
        y = np.array(input_skelleton[self.orientational_vector_joint_idxs_2[1]] - input_skelleton[self.orientational_vector_joint_idxs_2[0]])
        assert not np.array_equal(x, y)
        _x = x / np.linalg.norm(x)
        cross_x_y = np.cross(x,y)
        _z = cross_x_y / np.linalg.norm(cross_x_y)
        cross__z_x = np.cross(_z,x)
        _y = cross__z_x / np.linalg.norm(cross__z_x)
        return  np.stack([_x, _y, _z])

    def normalize_skelleton_size(self, input_skelleton):
        """Stretch all vectors of person to the same length as the vectors in normal_skelleton
        
        The order to build a completly new skeleton with the old vectors is defined by self.body_build_order. The first Index in the first tuple
        is the start point. It is the only point with the same coordinates as before. For the new skeleton, the algorithm
        takes the vector direction of person and the vecttor length of person2.
        """
        resized_skelleton = np.copy(input_skelleton)

        first_joint_idx = self.body_build_order[0][0]
        resized_skelleton[first_joint_idx] = np.copy(input_skelleton[first_joint_idx])

        for joint_idxs in self.body_build_order:
            joint1_p1 = input_skelleton[joint_idxs[0]]
            joint2_p1 = input_skelleton[joint_idxs[1]]

            joint1_p2 = self.normal_skelleton[joint_idxs[0]]
            joint2_p2 = self.normal_skelleton[joint_idxs[1]]
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


    def normalize_skelleton_position(self, input_skelleton):
        """Returns a skelleton with the pelvis at (0, 0, 0)."""
        reoriented_skelleton = np.copy(input_skelleton)
        # Move skelleton to 0/0/0 by substracting coordinates of central joint from all joints
        for joint_idx in range(len(reoriented_skelleton)):
            reoriented_skelleton[joint_idx][0] = input_skelleton[joint_idx][0] - input_skelleton[self.central_joint_idx][0]
            reoriented_skelleton[joint_idx][1] = input_skelleton[joint_idx][1] - input_skelleton[self.central_joint_idx][1]
            reoriented_skelleton[joint_idx][2] = input_skelleton[joint_idx][2] - input_skelleton[self.central_joint_idx][2]

        return reoriented_skelleton

    def normalize_skelleton_orientation(self, input_skelleton):
        """Returns a skelleton with the pevlis turned into the same directions as the normal skelleton."""
        reoriented_skelleton = np.copy(input_skelleton)
        reoriented_skelleton_rotation_matrix = self.calculate_orientation_vector(reoriented_skelleton)
        rotation_matrix_to_normal_orientation = np.matmul(self.normal_orientation_matrix_transpose, reoriented_skelleton_rotation_matrix)

        for joint_idx in range(len(reoriented_skelleton)):
            reoriented_skelleton[joint_idx] = np.matmul(reoriented_skelleton[joint_idx], rotation_matrix_to_normal_orientation)

        return reoriented_skelleton

    def skelleton_coherency_test(self):
        """Check whether all joints are connected."""
        already_added_nodes : Set[int] = set({})
        already_added_nodes.add(self.body_build_order[0][0])
        for joint in self.body_build_order:
            if joint[0] not in already_added_nodes:
                rp.logerr(f"ERROR: Index {joint[0]} (first index) was not added in the Set before")
            if joint[1] in already_added_nodes:
                rp.logerr(f"ERROR: Index {joint[0]} (second index) was added in the Set before")
            already_added_nodes.add(joint[1])
        rp.logerr(f"Total number of joints: {len(already_added_nodes)}")
        rp.logerr(already_added_nodes)

    def normalize_skelleton(self, input_skelleton):
        """Utility method to normalize size, position and orientation of skelleton."""
        skelleton = self.normalize_skelleton_size(input_skelleton)
        skelleton = self.normalize_skelleton_position(skelleton)
        skelleton = self.normalize_skelleton_orientation(skelleton)
        return skelleton

    def find_inner_joint(self, joint_names):
        """A triplet of three joints has three angles. This method finds the inner joint for an angle.

        The inner and outer joints dictionary returned by this method has the following form:
        Args:
            joint_names

        Returns:
            Name of the inner joint
        """
        # TODO: Optimize this if it has to be done more often
        joint_idxs = set(self.joints_used[self.get_joint_index(j)] for j in joint_names)

        def go_deeper(leaf, other_joints, building_blocks_left, found_joints):
            if found_joints == other_joints:
                return found_joints
            new_leafs = []
            for i, j in building_blocks_left:
                if i == leaf:
                    building_blocks_left = list(filter(lambda x: (x[0], x[1]) != (i, j), building_blocks_left))
                    new_leafs.append(j)
                if j == leaf:
                    building_blocks_left = list(filter(lambda x: (x[0], x[1]) != (i, j), building_blocks_left))
                    new_leafs.append(i)
                   

            for j in other_joints:
                if j in new_leafs:
                    found_joints.add(j)

            for l in new_leafs:
                found_joints.update(go_deeper(l, other_joints, building_blocks_left, found_joints))

            return found_joints

        body_build_order = [(self.joints_used[idx_1], self.joints_used[idx_2]) for idx_1, idx_2 in self.body_build_order]

        for joint in joint_idxs:
            total_found_leafs_no = 0
            other_joints = joint_idxs.copy()
            other_joints.remove(joint)
            leafs = set()
            initial_building_blocks = body_build_order.copy()
            for i, j in body_build_order:
                if joint == i:
                    initial_building_blocks.remove((i, j))
                    leafs.add(j)
                elif joint == j:
                    initial_building_blocks.remove((i, j))
                    leafs.add(i)

        
            for l in leafs:
                found_leafs = set()
                if l in other_joints:
                    found_leafs.add(l)
                found_leafs = go_deeper(l, other_joints, initial_building_blocks.copy(), found_leafs)
                found_leafs_no = len(found_leafs)
                if total_found_leafs_no == 1 and found_leafs_no == 1:
                    return self.get_joint_name(self.joints_used.index(joint))
                total_found_leafs_no += found_leafs_no


        raise IllegalAngleException("This angle is not defined, as there is no 'inner' joint.")
                

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



def resize_len_vec1_to_vec2(vec1, vec2):
    length_vec1 = np.linalg.norm(vec1)
    length_vec2 = np.linalg.norm(vec2)
    return (length_vec2 / length_vec1) * vec1
    
    
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