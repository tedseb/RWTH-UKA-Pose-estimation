import numpy as np
import math
# help cross product
# x = np.cross(y, z)
# y = np.cross(z, x)
# z = np.cross(x, y)


COMPARE_ANGLES = [
    [("LeftUpperLeg", 1), ("LeftLowerLeg", 4), ("LeftFoot", 7)],
    [("RightUpperLeg", 2), ("RightLowerLeg", 5), ("RightFoot", 8)]
]


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)) * (180 / np.pi)


def check_rotation_matrix(M, orth_thresh=1e-5, det_thresh=1e-5):
    if np.linalg.norm(M[:3, :3].T @ M[:3, :3] - np.eye(3)) > orth_thresh:
        print(M[:3, :3].T @ M[:3, :3])
        raise ValueError('Input matrix is not a pure rotation')
    if np.abs(np.linalg.det(M[:3, :3]) - 1.0) > det_thresh:
        raise ValueError('Input matrix is not a pure rotation')


def rotation_matrix_from_vectors(vec1, vec2):
    """
    Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix


def compute_angles(p1, p2, p3):
    a = p1 - p2
    b = p3 - p2
    xy = np.arctan2(a[0], a[1]) - np.arctan2(b[0], b[1])
    xz = np.arctan2(a[0], a[2]) - np.arctan2(b[0], b[2])
    yz = np.arctan2(a[1], a[2]) - np.arctan2(b[1], b[2])
    xy = np.absolute(xy) * (180 / np.pi)
    xz = np.absolute(xz) * (180 / np.pi)
    yz = np.absolute(yz) * (180 / np.pi)
    xy = 180 - abs(xy - 180)
    yz = 180 - abs(yz - 180)
    yz = 180 - abs(yz - 180)
    angle = angle_between(a, b)
    return (xy, xz, yz, angle)


def compute_angle_distance(self, point_positions1, point_positions2):
    """ computes angle diff of compared angles """
    for angle in COMPARE_ANGLES:
        # print(f" -- {angle} -- ")
        p1_1 = np.array(point_positions1[angle[0][0]])
        p1_2 = np.array(point_positions1[angle[1][0]])
        p1_3 = np.array(point_positions1[angle[2][0]])
        p2_1 = np.array(point_positions2[angle[0][1]])
        p2_2 = np.array(point_positions2[angle[1][1]])
        p2_3 = np.array(point_positions2[angle[2][1]])
        xy1, xz1, yz1, angle1 = compute_angles(p1_1, p1_2, p1_3)
        xy2, xz2, yz2, angle2 = compute_angles(p2_1, p2_2, p2_3)
        xy_diff = abs(xy1 - xy2)
        xz_diff = abs(xz1 - xz2)
        yz_diff = abs(yz1 - yz2)
        angle_diff = abs(angle1 - angle2)
        print(xy_diff, xz_diff, yz_diff, angle_diff)


def compute_euclidean_distance(mapping, point_positions1, point_positions2):
    """" Computes Average Distance of mapped points """
    distances = 0
    for mapping in mapping:
        pos1 = np.array(point_positions1[mapping[0]])
        pos2 = np.array(point_positions2[mapping[1]])
        distances += np.linalg.norm(pos1 - pos2)
    euclidean_distance = distances / len(mapping)
    print("### Average Distance Mapping ###")
    return euclidean_distance


def inverse_rotation_zxy(M, thresh=0.9999999):
    check_rotation_matrix(M)
    if np.abs(M[1, 2]) > thresh:
        sx = np.sign(-M[1, 2])
        x0 = sx * np.pi / 2

        # arbitrarily set z=0
        y0 = 0  # so sy=0, cy=1

        # compute x = arctan2( M[0,1]/sy, M[02]/sy )
        z0 = np.arctan2(M[2, 0] / sx, M[2, 1] / sx)
        return np.array((x0, y0, z0)), np.array((x0, y0, z0))
    else:
        x0 = np.arcsin(-M[1, 2])
        x1 = np.pi - x0
        c0 = np.cos(x0)
        c1 = np.cos(x1)

        y0 = np.arctan2(M[0, 2] / c0, M[2, 2] / c0)
        y1 = np.arctan2(M[0, 2] / c1, M[2, 2] / c1)
        z0 = np.arctan2(M[1, 0] / c0, M[1, 1] / c0)
        z1 = np.arctan2(M[1, 0] / c1, M[1, 1] / c1)
        return np.array((x0, y0, z0)), np.array((x1, y1, z1))


def inverse_rotation_xzy( M, thresh=0.9999999 ):
    check_rotation_matrix( M )
    if np.abs(M[1,0]) > thresh:
        sz = np.sign(M[1,0])
        z0 = sz*np.pi/2

        # arbitrarily set z=0
        y0 = 0 # so sy=0, cy=1

        # compute x = arctan2( M[0,1]/sy, M[02]/sy )
        x0 = np.arctan2( M[0,2]/sz, -M[0,1]/sz )
        return np.array((x0,y0,z0)), np.array((x0,y0,z0))
    else:
        z0 = np.arcsin( M[1,0] )
        z1 = np.pi - z0
        c0 = np.cos(z0)
        c1 = np.cos(z1)

        x0 = np.arctan2( -M[1,2]/c0, M[1,1]/c0 )
        x1 = np.arctan2( -M[1,2]/c1, M[1,1]/c1 )

        y0 = np.arctan2( -M[2,0]/c0, M[0,0]/c0 )
        y1 = np.arctan2( -M[2,0]/c1, M[0,0]/c1 )
        return np.array((x0,y0,z0)), np.array((x1,y1,z1))


def calculate_knee_rotation(hip, knee, foot, toe=None):
    """
    Computes the rotation from hip to knee in a rotation matrix and decompose it into an euler sequence
    for that it computes The basis B_hip for the hip like:
    x_hip = direction pointing to the left of a frontal person. cros product (knee->hip) with (knee->foot)
    y_hip = direction pointing up the tigh. := (knee->hip)
    z_hip  = direction to the front. cross product between x and y vector.

    The base of the knee is similar to the base of the hip. But y is pointing from foot->knee instead of knee->hip

    rotation M is the rotation matrix from a Point in the local coordinate system B_hip to the system in B_knee.
    Let x bee a point in world coordinates:
    # M * B_hip * x = B_knee * x
    # M * B_hip = B_knee
    # M * B_hip * B_hip^-1 = B_knee * B_hip^-1  | Rotation matrix properties: inverse =: transpose
    # M = B_knee * B_hip^T

    :param hip:     numpy array of  point hip
    :param knee:    numpy array of  point knee
    :param foot:    numpy array of  point foot
    :return:        euler_sequence: numpy array zxy rotation from hip to knee
    """
    y_d_knee = knee - foot
    z_d_knee = np.cross(knee - hip, y_d_knee)
    x_d_knee = np.cross(y_d_knee, z_d_knee)

    y_d_knee = y_d_knee / np.linalg.norm(y_d_knee)
    x_d_knee = x_d_knee / np.linalg.norm(x_d_knee)
    z_d_knee = z_d_knee / np.linalg.norm(z_d_knee)

    z_d_hip = z_d_knee
    y_d_hip = hip - knee
    x_d_hip = np.cross(y_d_hip, z_d_hip)
    z_d_hip = np.cross(x_d_hip, y_d_hip)
    z_d_hip = z_d_hip / np.linalg.norm(z_d_hip)
    y_d_hip = y_d_hip / np.linalg.norm(y_d_hip)
    x_d_hip = x_d_hip / np.linalg.norm(x_d_hip)

    basis_knee = np.array([x_d_knee, y_d_knee, z_d_knee])
    basis_hip = np.array([x_d_hip, y_d_hip, z_d_hip])

    check_rotation_matrix(basis_knee)
    check_rotation_matrix(basis_hip)

    basis_rotation = basis_knee @ np.transpose(basis_hip)

    check_rotation_matrix(basis_rotation)
    return inverse_rotation_zxy(basis_rotation)


def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


def calculate_ankle_rotation(hip, knee, foot, toes, default_deviation=0.41):
    y_d_knee = knee - foot
    z_d_knee = np.cross(knee - hip, y_d_knee)
    x_d_knee = np.cross(y_d_knee, z_d_knee)

    y_d_knee = y_d_knee / np.linalg.norm(y_d_knee)
    x_d_knee = x_d_knee / np.linalg.norm(x_d_knee)
    z_d_knee = z_d_knee / np.linalg.norm(z_d_knee)

    x_d_foot = toes - foot
    z_d_foot = np.cross(x_d_foot, y_d_knee)

    x_d_foot = np.dot(rotation_matrix(z_d_foot, default_deviation), x_d_foot)
    y_d_foot = np.cross(z_d_foot, x_d_foot)
    # z_d_foot = np.cross(x_d_foot, y_d_foot)

    z_d_foot = z_d_foot / np.linalg.norm(z_d_foot)
    y_d_foot = y_d_foot / np.linalg.norm(y_d_foot)
    x_d_foot = x_d_foot / np.linalg.norm(x_d_foot)

    basis_knee = np.array([x_d_knee, y_d_knee, z_d_knee])
    basis_foot = np.array([x_d_foot, y_d_foot, z_d_foot])

    check_rotation_matrix(basis_knee)
    check_rotation_matrix(basis_foot)

    # basis_rotation = np.transpose(basis_foot) @ basis_foot
    basis_rotation = basis_foot @ np.transpose(basis_knee)
    check_rotation_matrix(basis_rotation)
    return np.absolute(inverse_rotation_zxy(basis_rotation))


def calculate_pelvis_rotation(knee, right_hip, pelvis, l5):
    z_d_hip = right_hip - pelvis
    y_d_hip = right_hip - knee
    x_d_hip = np.cross(y_d_hip, z_d_hip)
    z_d_hip = np.cross(x_d_hip, y_d_hip)
    z_d_hip = z_d_hip / np.linalg.norm(z_d_hip)
    y_d_hip = y_d_hip / np.linalg.norm(y_d_hip)
    x_d_hip = x_d_hip / np.linalg.norm(x_d_hip)

    z_d_pelvis = z_d_hip
    y_d_pelvis = l5 - pelvis
    x_d_pelvis = np.cross(y_d_pelvis, z_d_pelvis)
    # y_d_pelvis = np.cross(x_d_pelvis, z_d_pelvis)
    z_d_pelvis = np.cross(x_d_pelvis, y_d_pelvis)
    y_d_pelvis = y_d_pelvis / np.linalg.norm(y_d_pelvis)
    x_d_pelvis = x_d_pelvis / np.linalg.norm(x_d_pelvis)
    z_d_pelvis = z_d_pelvis / np.linalg.norm(z_d_pelvis)

    basis_hip = np.array([x_d_hip, y_d_hip, z_d_hip])
    basis_pelvis = np.array([x_d_pelvis, y_d_pelvis, z_d_pelvis])

    check_rotation_matrix(basis_hip)
    check_rotation_matrix(basis_pelvis)

    basis_rotation = basis_pelvis @ np.transpose(basis_hip)
    check_rotation_matrix(basis_rotation)
    return inverse_rotation_zxy(basis_rotation)


def calculate_ergo_upper_arm_rotation(t8, neck, shoulder_right, upper_arm_right, fore_arm_right):
    z_d_t8 = upper_arm_right - shoulder_right
    y_d_t8 = neck - t8
    x_d_t8 = np.cross(y_d_t8, z_d_t8)
    z_d_t8 = np.cross(x_d_t8, y_d_t8)
    z_d_t8 = z_d_t8 / np.linalg.norm(z_d_t8)
    y_d_t8 = y_d_t8 / np.linalg.norm(y_d_t8)
    x_d_t8 = x_d_t8 / np.linalg.norm(x_d_t8)

    z_d_upper_arm = z_d_t8
    y_d_upper_arm = upper_arm_right - fore_arm_right
    x_d_upper_arm = np.cross(y_d_upper_arm, z_d_upper_arm)
    # y_d_upper_arm = np.cross(x_d_upper_arm, z_d_upper_arm)
    z_d_upper_arm = np.cross(x_d_upper_arm, y_d_upper_arm)
    y_d_upper_arm = y_d_upper_arm / np.linalg.norm(y_d_upper_arm)
    x_d_upper_arm = x_d_upper_arm / np.linalg.norm(x_d_upper_arm)
    z_d_upper_arm = z_d_upper_arm / np.linalg.norm(z_d_upper_arm)

    basis_t8 = np.array([x_d_t8, y_d_t8, z_d_t8])
    basis_upper_arm = np.array([x_d_upper_arm, y_d_upper_arm, z_d_upper_arm])

    check_rotation_matrix(basis_t8)
    check_rotation_matrix(basis_upper_arm)

    basis_rotation = basis_t8 @ np.transpose(basis_upper_arm)
    check_rotation_matrix(basis_rotation)
    return inverse_rotation_zxy(basis_rotation)


def calculate_ergo_upper_arm_rotation2(t8, neck, shoulder_right, upper_arm_right, fore_arm_right):
    z_d_t8 = upper_arm_right - shoulder_right
    y_d_t8 = neck - t8
    x_d_t8 = np.cross(y_d_t8, z_d_t8)
    z_d_t8 = np.cross(x_d_t8, y_d_t8)
    x_d_t8 = x_d_t8 / np.linalg.norm(x_d_t8)
    y_d_t8 = y_d_t8 / np.linalg.norm(y_d_t8)
    z_d_t8 = z_d_t8 / np.linalg.norm(z_d_t8)
    basis_t8 = np.array([x_d_t8, y_d_t8, z_d_t8])
    check_rotation_matrix(basis_t8)

    vec1 = basis_t8 @ (neck - t8)
    vec2 = basis_t8 @ (upper_arm_right - fore_arm_right)

    basis_rotation = rotation_matrix_from_vectors(vec1, vec2)
    check_rotation_matrix(basis_rotation)
    return inverse_rotation_zxy(basis_rotation)


def calculate_upper_arm_rotation(t8, neck, shoulder_right, upper_arm_right, fore_arm_right, hand_right):
    z_d_upper_arm = upper_arm_right - shoulder_right
    y_d_upper_arm = neck - t8
    x_d_upper_arm = np.cross(y_d_upper_arm, z_d_upper_arm)
    # y_d_upper_arm = np.cross(x_d_upper_arm, z_d_upper_arm)
    z_d_upper_arm = np.cross(x_d_upper_arm, y_d_upper_arm)
    x_d_upper_arm = x_d_upper_arm / np.linalg.norm(x_d_upper_arm)
    y_d_upper_arm = y_d_upper_arm / np.linalg.norm(y_d_upper_arm)
    z_d_upper_arm = z_d_upper_arm / np.linalg.norm(z_d_upper_arm)

    x_d_elbow = hand_right - fore_arm_right
    y_d_elbow = upper_arm_right - fore_arm_right
    z_d_elbow = np.cross(x_d_elbow, y_d_elbow)
    x_d_elbow = np.cross(y_d_elbow, z_d_elbow)
    x_d_elbow = x_d_elbow / np.linalg.norm(x_d_elbow)
    y_d_elbow = y_d_elbow / np.linalg.norm(y_d_elbow)
    z_d_elbow = z_d_elbow / np.linalg.norm(z_d_elbow)

    basis_elbow = np.array([x_d_elbow, y_d_elbow, z_d_elbow])
    basis_upper_arm = np.array([x_d_upper_arm, y_d_upper_arm, z_d_upper_arm])

    check_rotation_matrix(basis_elbow)
    check_rotation_matrix(basis_upper_arm)

    basis_rotation = basis_upper_arm @ np.transpose(basis_elbow)
    check_rotation_matrix(basis_rotation)
    return inverse_rotation_zxy(basis_rotation)


# def calculate_ergo_pelvis_rotation(hip_right, pelvis, L5, up_vector=np.array([0.0, 0.0, 1.0]), rotate=None):
#     z_d_pelvis = hip_right - pelvis
#     y_d_pelvis = L5 - pelvis
#     x_d_pelvis = np.cross(z_d_pelvis, y_d_pelvis)
#     z_d_pelvis = np.cross(x_d_pelvis, y_d_pelvis)

#     x_d_pelvis = x_d_pelvis / np.linalg.norm(x_d_pelvis)
#     y_d_pelvis = y_d_pelvis / np.linalg.norm(y_d_pelvis)
#     z_d_pelvis = z_d_pelvis / np.linalg.norm(z_d_pelvis)

#     basis_pelvis = np.array([x_d_pelvis, y_d_pelvis, z_d_pelvis])
#     check_rotation_matrix(basis_pelvis)
#     vec1 = basis_pelvis @ up_vector
#     vec2 = basis_pelvis @ y_d_pelvis

#     basis_rotation = rotation_matrix_from_vectors(vec1, vec2)
#     check_rotation_matrix(basis_rotation)
#     rotation = inverse_rotation_zxy(basis_rotation)
#     if rotate:
#         rotation[0][2] += rotate / (180 / np.pi)
#     return rotation


def calculate_ergo_pelvis_rotation(hip_right, pelvis, L5, up_vector=np.array([0.0, 0.0, 1.0]), rotate=None):
    z_d_pelvis = hip_right - pelvis
    y_d_pelvis = L5 - pelvis
    x_d_pelvis = np.cross(z_d_pelvis, y_d_pelvis)
    z_d_pelvis = np.cross(x_d_pelvis, y_d_pelvis)

    x_d_pelvis = x_d_pelvis / np.linalg.norm(x_d_pelvis)
    y_d_pelvis = y_d_pelvis / np.linalg.norm(y_d_pelvis)
    z_d_pelvis = z_d_pelvis / np.linalg.norm(z_d_pelvis)

    basis_pelvis = np.array([x_d_pelvis, y_d_pelvis, z_d_pelvis])
    check_rotation_matrix(basis_pelvis)
    vec1 = basis_pelvis @ up_vector
    vec2 = basis_pelvis @ y_d_pelvis

    basis_rotation = rotation_matrix_from_vectors(vec1, vec2)
    check_rotation_matrix(basis_rotation)
    rotation = inverse_rotation_zxy(basis_rotation)
    if rotate:
        rotation[0][2] += rotate / (180 / np.pi)
    return rotation
