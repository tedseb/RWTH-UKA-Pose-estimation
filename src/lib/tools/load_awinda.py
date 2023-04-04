import copy
import pickle
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import procrustes
import rospy
import tensorflow as tf
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# PATH_MVNX = "data/awinda/Test-007#Hannah2.mvnx"
# PATH_VIDEO = "data/awinda/Test-007_drehen#Hannah2.mp4"
# PATH_VIDEO = "data/double_squats_02.mp4"
# PATH_VIDEO = "data/videos/Uv9qwcALYKU_720p.mp4"

VIDEO_CONFIG_HAMPELMANN_VORN = {
    "path_video": "data/awinda/hampelmann_vorne/Julia-014.mp4",
    "path_mvnx": "data/awinda/hampelmann_vorne/Hampelmann_vorne.mvnx",
    "frame_delay": 41,
    "measure_start": 380,
    "measure_end": 1000
}

VIDEO_CONFIG_KNIEBEUGEN_VORN = {
    "path_video": "data/awinda/kniebeugen_vorne/Julia-008.mp4",
    "path_mvnx": "data/awinda/kniebeugen_vorne/Kniebeuge_vorne.mvnx",
    "frame_delay": 34,
    "measure_start": 354,  # (344, Stehen)
    "measure_end": 1742,  # (408, Stehen)
    "data_name": "kniebeuge_front"
}

VIDEO_CONFIG_KNIEBEUGEN_VORN_RECHTS = {
    "path_video": "data/awinda/kniebeugen_vorne_rechts/Julia-010.mp4",
    "path_mvnx": "data/awinda/kniebeugen_vorne_rechts/Kniebeugen_vorne_rechts.mvnx",
    "frame_delay": 34,
    "measure_start": 518,
    "measure_end": 2078,
    "data_name": "kniebeuge_side"
}

VIDEO_CONFIG_STEHEN_VORN = {
    "path_video": "data/awinda/stehen_verschieden/Julia-006.mp4",
    "path_mvnx": "data/awinda/stehen_verschieden/Stehen_verschieden.mvnx",
    "frame_delay": 34,
    "measure_start": 644,
    "measure_end": 1820,
    "data_name": "stehen"
}

VIDEO_CONFIG_GEHEN_VORN = {
    "path_video": "data/awinda/gehen_vorne/Julia-005.mp4",
    "path_mvnx": "data/awinda/gehen_vorne/gehen_vorne_hinten.mvnx",
    "frame_delay": 34,
    "measure_start": 324,
    "measure_end": 1062,
    "data_name": "gehen_front"
}

VIDEO_CONFIG_GEHEN_SIDE_LEFT = {
    "path_video": "data/awinda/gehen_links/Julia-029.mp4",
    "path_mvnx": "data/awinda/gehen_links/gehen_links.mvnx",
    "frame_delay": 36,
    "measure_start": 36,
    "measure_end": 210,
    "rotate": True,
    "data_name": "gehen_left_front"
}

VIDEO_CONFIG_GEHEN_SIDE_RIGHT = {
    "path_video": "data/awinda/gehen_rechts/Julia-028.mp4",
    "path_mvnx": "data/awinda/gehen_rechts/gehen_rechts.mvnx",
    "frame_delay": 36,
    "measure_start": 36,
    "measure_end": 210,
    "rotate": True,
    "data_name": "gehen_right_front"
}

VIDEO_CONFIG = VIDEO_CONFIG_GEHEN_SIDE_LEFT
METRABS_PATH = "/home/trainerai/trainerai-core/src/AI/metrabs/models/metrabs_rn34_y4"
# METRABS_PATH = "/home/trainerai/trainerai-core/src/AI/metrabs/models/metrabs_eff2m_y4"


USE_METRABS = True
COMPARE_ANGLES = [
    [("LeftUpperLeg", 1), ("LeftLowerLeg", 4), ("LeftFoot", 7)],
    [("RightUpperLeg", 2), ("RightLowerLeg", 5), ("RightFoot", 8)]
]

SMPL_CONNECTIONS = [
    (1, 4), (1, 0), (2, 5), (2, 0), (3, 6), (3, 0), (4, 7), (5, 8), (6, 9), (7, 10), (8, 11), (9, 12), (12, 13),
    (12, 14), (12, 15), (13, 16), (14, 17), (16, 18), (17, 19), (18, 20), (19, 21), (20, 22), (21, 23)
]

# Hips, Knees
MAPPINGS_0 = [
    ("Neck", 12),
    ("LeftUpperArm", 16),
    ("RightUpperArm", 17),
    ("LeftLowerLeg", 4),
    ("RightLowerLeg", 5),
    ("LeftFoot", 7),
    ("RightFoot", 8),
    ("LeftForeArm", 18),
    ("RightForeArm", 19)
]

# Hips, Neck, Head
MAPPINGS_1 = [("Pelvis", 0), ("Neck", 12), ("Head", 15)]

# Hips, Shoulders
MAPPINGS_2 = [("Neck", 12), ("LeftUpperArm", 16), ("RightUpperArm", 17), ("LeftLowerLeg", 4), ("RightLowerLeg", 5)]

MAPPING = [
    [5, 11],
    [12, 15],
    [8, 16],
    [19, 48],
    [15, 96],
    [18, 10],
    [22, 9],
    [13, 39],
    [9, 87]
]

TEST_CONNECTIONS = [
    [74, 48],
    [74, 96],
    [48, 52],
    [96, 100],
    [52, 31],
    [100, 79],
    [31, 9],
    [79, 10],
    [74, 2],
    [2, 120],
    [120, 68],
    [68, 11],
    [11, 25],
    [68, 82],
    [68, 34],
    [82, 16],
    [34, 15],
    [16, 87],
    [15, 39],
    [87, 22],
    [39, 21]
]

SKELETON_INDICES = {
    "coco_19": [68, 72, 74, 56, 37, 64, 48, 52, 30, 104, 85, 112, 96, 100, 78, 41, 35, 89, 83],
    "smpl_24": [23, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
}


def check_rotation_matrix(M, orth_thresh=1e-5, det_thresh=1e-5):
    if np.linalg.norm(M[:3, :3].T @ M[:3, :3] - np.eye(3)) > orth_thresh:
        print(M[:3, :3].T @ M[:3, :3])
        raise ValueError('Input matrix is not a pure rotation')
    if np.abs(np.linalg.det(M[:3, :3]) - 1.0) > det_thresh:
        raise ValueError('Input matrix is not a pure rotation')


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)) * (180 / np.pi)


# Todo: Import this from lib
class TraceTime:
    def __init__(self, name):
        self._name = name
        self._time_stamp = 0

    def __enter__(self):
        self._time_stamp = time.time()

    def __exit__(self, type, value, traceback):
        time_elapsed = (time.time() - self._time_stamp) * 1000
        # print(f"Time {self._name}: {time_elapsed}ms")


class AwindaDataToRos:
    def __init__(self):
        rospy.init_node('awinda_data', anonymous=False)
        transform_path = "/home/trainerai/trainerai-core/src/station_manager/launch/static_transform.launch"
        self._transform1 = subprocess.Popen(["roslaunch", transform_path, "dev:=0"])
        self._transform2 = subprocess.Popen(["roslaunch", transform_path, "dev:=1"])

        self._skeleton_pub0 = rospy.Publisher("/visualization/skeleton_0", MarkerArray, queue_size=5)
        self._skeleton_pub1 = rospy.Publisher("/visualization/skeleton_1", MarkerArray, queue_size=5)
        self._image_pub = rospy.Publisher("/image/channel_0_yolo", Image, queue_size=5)
        self._skeleton_scale = Vector3(0.03, 0.03, 0.03)
        self._connection_scale = Vector3(0.01, 0.01, 0.01)
        self._scale = 1
        self._cap = None
        self._model = None
        self._mappings = None
        self._angle_diff = {}
        self._angle_xsens = {}
        self._angle_metrabs = {}
        self._angles_count = 0
        self._max_frame = 0
        self._frame_counter = 0
        self._frame_index = 0
        self._frame_video_diff = -1
        if USE_METRABS:
            self.init_metrabs()
        self.open_video(VIDEO_CONFIG["path_video"])

    def __del__(self):
        self._transform1.terminate()
        try:
            self._transform1.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.kill(self._transform1.pid)
        del self._transform1

        self._transform2.terminate()
        try:
            self._transform2.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.kill(self._transform2.pid)
        del self._transform2
        rospy.signal_shutdown("End")

    def init_metrabs(self):
        self._model = tf.saved_model.load(METRABS_PATH)

    def send_metrabs(self, image, awinda_reference):
        if self._model is None:
            return False

        with TraceTime("Metrabs"):
            pred = self._model.detect_poses(
                image, skeleton='', default_fov_degrees=55, detector_threshold=0.5)

        positions = pred['poses3d'].numpy()
        if len(positions) == 0:
            return False, ""

        scale = 0.01
        positions = positions[0]
        positions = np.array(list(map(lambda x: x * scale, positions)))


        awinda_refs = np.array([awinda_reference[x[0]] for x in self._mappings])
        metrabs_refs = np.array([positions[x[1]] for x in self._mappings])


        with TraceTime("Mapping"):
            d, Z, tform = procrustes.procrustes(awinda_refs, metrabs_refs, True, False)
        T = tform['rotation']
        b = tform['scale']
        c = tform['translation']

        positions = b * positions @ T + c
        indices = SKELETON_INDICES["smpl_24"]
        # connections = list(map(lambda x: (indices[x[0]], indices[x[1]]), TEST_CONNECTIONS))
        connections = list(map(lambda x: (x[0], x[1]), TEST_CONNECTIONS))

        self.send_ros_markers(positions, connections, "dev1", ColorRGBA(0.30, 0.98, 0.30, 1.00))
        return positions, tform

    @staticmethod
    def read_awinda_mvnx(file_name: str):
        start = time.time()
        root = ET.parse(file_name).getroot()
        end = time.time()
        print("read in ", end - start)
        return root

    @staticmethod
    def get_points_and_connections(root):
        points = []
        segments = root.find("{http://www.xsens.com/mvn/mvnx}subject").find("{http://www.xsens.com/mvn/mvnx}segments")
        for segment in segments:
            points.append(segment.attrib["label"])
        joints = root.find("{http://www.xsens.com/mvn/mvnx}subject").find("{http://www.xsens.com/mvn/mvnx}joints")
        connections = []
        for joint in joints:
            connection = []
            for connector in joint:
                connector_joint_name = connector.text.split("/")[0]
                connection.append(points.index(connector_joint_name))
            connections.append(connection)
        return (points, connections)

    @staticmethod
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
        for angle in COMPARE_ANGLES:
            # print(f" -- {angle} -- ")
            p1_1 = np.array(point_positions1[angle[0][0]])
            p1_2 = np.array(point_positions1[angle[1][0]])
            p1_3 = np.array(point_positions1[angle[2][0]])
            p2_1 = np.array(point_positions2[angle[0][1]])
            p2_2 = np.array(point_positions2[angle[1][1]])
            p2_3 = np.array(point_positions2[angle[2][1]])
            xy1, xz1, yz1, angle1 = self.compute_angles(p1_1, p1_2, p1_3)
            xy2, xz2, yz2, angle2 = self.compute_angles(p2_1, p2_2, p2_3)
            xy_diff = abs(xy1 - xy2)
            xz_diff = abs(xz1 - xz2)
            yz_diff = abs(yz1 - yz2)
            angle_diff = abs(angle1 - angle2)
            print(xy_diff, xz_diff, yz_diff, angle_diff)

    def compute_euclidean_distance(self, point_positions1, point_positions2):
        distances = 0
        for mapping in MAPPING:
            pos1 = np.array(point_positions1[mapping[0]])
            pos2 = np.array(point_positions2[mapping[1]])
            distances += np.linalg.norm(pos1 - pos2)
        self._euclidean_distance = distances / len(MAPPING)
        self._euclidean_count += 1
        print("### Average Distance Mapping ###")
        print(self._euclidean_distance)

    def print_angle_result(self):
        diff_angles = [[x[0], x[1], x[2]] for x in self._angle_diff.values()]
        average_angles = np.average(diff_angles, axis=0)
        median_angles = np.median(diff_angles, axis=0)
        min_angles = np.min(diff_angles, axis=0)
        max_angles = np.max(diff_angles, axis=0)

        print("Average:", f"X={average_angles[0]}", f"Y={average_angles[1]}", f"Z={average_angles[2]}")
        print("Median:", f"X={median_angles[0]}", f"Y={median_angles[1]}", f"Z={median_angles[2]}")
        print("Min:", f"X={min_angles[0]}", f"Y={min_angles[1]}", f"Z={min_angles[2]}")
        print("Max:", f"X={max_angles[0]}", f"Y={max_angles[1]}", f"Z={max_angles[2]}")

        data = {
            "diff": self._angle_diff,
            "metrabs": self._angle_metrabs,
            "xsens": self._angle_xsens,
            "path_video": VIDEO_CONFIG["path_video"],
            "frame_video_diff": self._frame_video_diff
        }
        print("Store", data["frame_video_diff"])

        data_name = VIDEO_CONFIG.get("data_name", 'angle_diff') + ".pickle"
        with open(data_name, 'wb') as f:
            pickle.dump(data, f)
        # print("MEAN:", f"X={x_mean}", f"Y={y_mean}", f"Z={z_mean}")
        print("Count:", len(diff_angles))

    def send_ros_markers(self, positions, connections, frame_id="dev0", color=ColorRGBA(0.98, 0.30, 0.30, 1.00)):
        idx = 0
        marker_array = MarkerArray()
        conecction_points = set({})
        for connection in connections:
            conecction_points.update(connection)

        for i, position in enumerate(positions):
            if i not in conecction_points:
                continue
            m = Marker()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = frame_id
            m.id = idx
            idx += 1
            m.ns = ''
            m.color = color
            m.scale = self._skeleton_scale
            m.pose.position.x, m.pose.position.y, m.pose.position.z = position
            m.type = 2
            m.action = 0
            m.lifetime = rospy.Duration(60)
            m_text = copy.deepcopy(m)
            m_text.id = idx
            idx += 1
            m_text.type = 9
            m_text.scale = Vector3(0.02, 0.02, 0.02)
            m_text.text = f"   {i}"
            # marker_array.markers.append(m_text)
            marker_array.markers.append(m)

        for connection in connections:
            m = Marker()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = frame_id
            m.id = idx
            idx += 1
            m.ns = ''
            m.color = color
            m.scale = self._connection_scale
            m.points = [Point(*positions[connection[0]]), Point(*positions[connection[1]])]
            m.type = 4
            m.action = 0
            m.lifetime = rospy.Duration(60)
            marker_array.markers.append(m)

        if frame_id == "dev0":
            self._skeleton_pub0.publish(marker_array)
        else:
            self._skeleton_pub1.publish(marker_array)

    @staticmethod
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

    @staticmethod
    def calculate_knee_rotation(hip, knee, foot, toe):
        y_d_knee = knee - foot
        z_d_knee = np.cross(toe - foot, y_d_knee)
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
        return AwindaDataToRos.inverse_rotation_zxy(basis_rotation)

    def store_euler_sequence(self, frame_counter, zxy_xsense, zxy_metrabs):
        xsense_knee = zxy_xsense[0] * (180 / np.pi)
        metrabs_knee = zxy_metrabs[0] * (180 / np.pi)
        self._angle_metrabs[frame_counter] = metrabs_knee
        self._angle_xsens[frame_counter] = xsense_knee
        diff_x = abs(float(xsense_knee[0]) - float(metrabs_knee[0]))
        diff_y = abs(float(xsense_knee[1]) - float(metrabs_knee[1]))
        diff_z = abs(float(xsense_knee[2]) - float(metrabs_knee[2]))
        angle_diff = (diff_x, diff_y, diff_z)
        self._angle_diff[frame_counter] = angle_diff
        self._angles_count += 1
        print("frame:", frame_counter, ", xsens:", xsense_knee, ", ours:", metrabs_knee)

    def extract_frame_data(self, frame):
        positions = frame.find("{http://www.xsens.com/mvn/mvnx}position")
        positions = positions.text.split()
        positions = list(map(lambda x: float(x) * self._scale, positions))
        joint_angle = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngle").text.split()
        joint_angle_xzy = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngleXZY").text.split()
        return (positions, joint_angle, joint_angle_xzy)

    def start(self, path: str):
        np.set_printoptions(suppress=True)
        np.set_printoptions(precision=3)
        root = self.read_awinda_mvnx(path)
        point_names, connections = self.get_points_and_connections(root)
        self._mappings = MAPPING  # list(map(lambda x: (point_names.index(x[0]), x[1]), MAPPINGS_0))

        point_posittions = dict(map(lambda x: (x, [0., 0., 0.]), point_names))
        frames = list(root.find("{http://www.xsens.com/mvn/mvnx}subject").find("{http://www.xsens.com/mvn/mvnx}frames"))
        start_time_ms = time.time() * 1000

        self._max_frame = len(frames)
        while self._frame_index < self._max_frame:

            frame = frames[self._frame_index]
            if self._frame_index <= VIDEO_CONFIG["frame_delay"]:
                self._frame_index += 1
                continue

            if frame.attrib["type"] != "normal":
                self._frame_index += 1
                continue

            if self._frame_index % 2 == 1:
                self._frame_index += 1
                continue

            positions, joint_angle, joint_angle_xzy = self.extract_frame_data(frame)

            angle_xsens = []
            angle_xzy_xsens = []
            for i in range(22):
                angle_xsens.append(joint_angle[i * 3:(i + 1) * 3])
                angle_xzy_xsens.append(joint_angle_xzy[i * 3:(i + 1) * 3])

            if len(positions) % 3 != 0:
                print("[ERROR]: positions are not divisible by 3")
                break

            position_len = len(positions) // 3
            for i in range(position_len):
                point_posittions[point_names[i]] = positions[i * 3:(i + 1) * 3]


            self.send_video()
            if USE_METRABS:
                awinda_pos = np.array(list(point_posittions.values()))
                metrabs_pos, tform = self.send_metrabs(self._cur_frame, awinda_pos)
                if np.any(metrabs_pos):
                    if VIDEO_CONFIG["measure_start"] <= self._frame_index <= VIDEO_CONFIG["measure_end"]:
                        zxy_xsense = self.calculate_knee_rotation(
                            np.array(point_posittions["RightUpperLeg"]),
                            np.array(point_posittions["RightLowerLeg"]),
                            np.array(point_posittions["RightFoot"]),
                            np.array(point_posittions["RightToe"])
                        )

                        zxy_metrabs = self.calculate_knee_rotation(
                            np.array(metrabs_pos[96]),
                            np.array(metrabs_pos[100]),
                            np.array(metrabs_pos[79]),
                            np.array(metrabs_pos[10])
                        )
                        self.store_euler_sequence(self._frame_index, zxy_xsense, zxy_metrabs)
                        # print(f"### {frame_counter} ###")
                        # self.compute_angle_distance(point_posittions, metrabs_pos)
                        # self.compute_euclidean_distance(awinda_pos, metrabs_pos)
                        # time.sleep(0.2)

                if self._frame_index == VIDEO_CONFIG["measure_end"]:
                    self.print_angle_result()
                    # self.print_euclidean_result()

                if self._frame_index == 378:
                    print("# save #")
                    with open('awinda_data.pickle', 'wb') as f:
                        data = {
                            "awinda_pos": list(point_posittions.values()),
                            "connections": connections,
                            "image": self._cur_frame,
                            "transformation": tform,
                            "frame_id": self._frame_index,
                            "angles": joint_angle,
                            "angles_xzy": joint_angle_xzy
                        }
                        pickle.dump(data, f)

            self.send_ros_markers(list(point_posittions.values()), connections)

            frame_time_stamp_ms = float(frame.attrib["time"])
            time_now_ms = time.time() * 1000
            time_diff_ms = time_now_ms - start_time_ms
            wait_time_s = max(frame_time_stamp_ms - time_diff_ms, 0) / 1000.
            # print("frame:", frame_time_stamp_ms, "stamp:", time_diff_ms, "wait:", wait_time_s)
            time.sleep(wait_time_s)
            self._frame_index += 1

            # if frame_counter > 500:
            #     break

    def open_video(self, path: str):
        self._cap = cv2.VideoCapture(path)

    def send_video(self):
        ret, self._cur_frame = self._cap.read()
        if VIDEO_CONFIG.get("rotate", False):
            self._cur_frame = cv2.rotate(self._cur_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow('img', self._cur_frame)


        if self._frame_video_diff == -1:
            self._frame_video_diff = self._frame_index - self._frame_counter
        self._frame_counter += 1

        speed = 5
        key = cv2.waitKey(1)
        if key == ord('a'):
            self._frame_index -= speed * 2
            self._frame_counter = max(self._frame_counter - speed, 0)
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, self._frame_counter)
            print("<<")
        if key == ord('d'):
            self._frame_index += speed * 2
            self._frame_counter = max(self._frame_counter + speed, 0)
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, self._frame_counter)
            print(">>")
        if key == ord(' '):
            print("Stop")
            cv2.waitKey(0)

        if not ret:
            return

        img = Image()
        img.header.stamp = rospy.Time.now()
        img.header.frame_id = "dev0"
        img.encoding = "bgr8"
        img.data = np.array(self._cur_frame, dtype=np.uint8).tobytes()
        img.height, img.width = self._cur_frame.shape[:-1]
        img.step = self._cur_frame.shape[-1] * self._cur_frame.shape[0]
        self._image_pub.publish(img)


def main():
    converter = AwindaDataToRos()

    def signal_handler(self, signal):
        print("############ SHUTDOWN ##################")
        nonlocal converter
        del converter
        sys.exit(0)

    time.sleep(2)
    signal.signal(signal.SIGINT, signal_handler)
    converter.start(VIDEO_CONFIG["path_mvnx"])


if __name__ == "__main__":
    main()
