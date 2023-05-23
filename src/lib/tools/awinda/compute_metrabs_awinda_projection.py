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
from video_configs import VIDEO_CONFIGS

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

VIDEO_CONFIGS = [
    VIDEO_CONFIGS.VIDEO_CONFIG_04_2023_KNIEBEUGEN,
    # VIDEO_CONFIGS.VIDEO_CONFIG_04_2023_LIEGESTUTZEN,
    # VIDEO_CONFIGS.VIDEO_CONFIG_04_2023_SITUPS
]
# METRABS_PATH = "/home/trainerai/trainerai-core/src/AI/metrabs/models/metrabs_rn34_y4"
METRABS_PATH = "/home/trainerai/trainerai-core/src/AI/metrabs/models/metrabs_eff2m_y4"


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

MAPPING = [
    [5, 11],
    [12, 15],
    [8, 16],
    [19, 49],
    [15, 96],
    [17, 78],
    # [21, 30],
    # [16, 103],
    [20, 55],
    [6, 25]
]

# Nacken, Knie, Ellenbogen
# MAPPING = [
#     [7, 82],
#     [11, 34],
#     [21, 32],
#     [17, 79],
# ]

# MAPPING = [
#     [19, 49],
#     [0, 75],
#     [15, 97],
#     [1, 2],
#     [8, 16],
#     [12, 15]
# ]

TEST_CONNECTIONS = [
    [77, 48],
    [77, 96],
    [48, 55],
    [96, 103],
    [55, 30],
    [103, 78],
    [30, 63],
    [78, 111],
    [77, 120],
    [120, 68],
    [68, 11],
    [11, 25],
    [68, 82],
    [68, 34],
    [82, 16],
    [34, 15],
    [16, 87],
    [15, 39],
    [87, 20],
    [39, 19]
]


# TEST_CONNECTIONS = [
#     [75, 49],
#     [75, 97],
#     [49, 52],
#     [97, 100],
#     [52, 31],
#     [100, 79],
#     [31, 9],
#     [79, 10],
#     [75, 2],
#     [2, 120],
#     [120, 68],
#     [68, 11],
#     [11, 25],
#     [68, 82],
#     [68, 34],
#     [82, 16],
#     [34, 15],
#     [16, 87],
#     [15, 39],
#     [87, 22],
#     [39, 21]
# ]

METRABS_DEPENDENCIES = {
    0: [96, 74, 48, 2, 120, 68],
    15: [96, 74, 48, 2, 120, 68],
    19: [96, 74, 48, 2, 120, 68],
    1: [96, 74, 48, 2, 120, 68],
    2: [96, 74, 48, 2, 120, 68, 82, 34],
    3: [96, 74, 48, 2, 120, 68, 82, 34],
    4: [74, 2, 120, 68, 82, 34, 16, 15],
    7: [120, 68, 82, 34, 16],
    8: [120, 68, 82, 16],
    11: [120, 68, 82, 34, 15],
    12: [120, 68, 34, 15],
    5: [68, 82, 34, 11, 25],
    6: [68, 82, 34, 11, 25],
}

SKELETON_INDICES = {
    "coco_19": [68, 72, 74, 56, 37, 64, 48, 52, 30, 104, 85, 112, 96, 100, 78, 41, 35, 89, 83],
    "smpl_24": [23, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
}

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
    def __init__(self, use_metrab=True):
        rospy.init_node('awinda_data', anonymous=False)
        transform_path = "/home/trainerai/trainerai-core/src/station_manager/launch/static_transform.launch"
        self._transform1 = subprocess.Popen(["roslaunch", transform_path, "dev:=0"])
        self._transform2 = subprocess.Popen(["roslaunch", transform_path, "dev:=1"])

        self._skeleton_pub0 = rospy.Publisher("/visualization/skeleton_0", MarkerArray, queue_size=5)
        self._skeleton_pub1 = rospy.Publisher("/visualization/skeleton_1", MarkerArray, queue_size=5)
        self._image_pub = rospy.Publisher("/image/channel_0_yolo", Image, queue_size=5)
        self._skeleton_scale = Vector3(0.01, 0.01, 0.01)
        self._connection_scale = Vector3(0.0025, 0.0025, 0.0025)
        self._scale = 1
        self._cap = None
        self._model = None
        self._mappings = None
        self._scales = []

        self._max_frame = 0
        self._frame_counter = 0
        self._frame_index = 0
        self._skip_frames = 1
        self._awinda_connections = None
        self._snapshots = []  # Array of image, awinda_pos, metrabs_pos
        if use_metrab:
            self.init_metrabs()

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


    def get_mapped_position(self, awinda_positions, metrabs_positions):
        awinda_refs = np.array([awinda_positions[x[0]] for x in self._mappings])
        metrabs_refs = np.array([metrabs_positions[x[1]] for x in self._mappings])

        with TraceTime("Mapping"):
            d, Z, tform = procrustes.procrustes(awinda_refs, metrabs_refs, True, False)
        T = tform['rotation']
        b = tform['scale']
        c = tform['translation']
        self._scales.append(b)

        positions = b * metrabs_positions @ T + c
        return positions

    def get_metrabs(self, image, awinda_reference, mapped=False):
        if self._model is None:
            return False

        with TraceTime("Metrabs"):
            pred = self._model.detect_poses(
                image, skeleton='', default_fov_degrees=55, detector_threshold=0.5)

        positions = pred['poses3d'].numpy()
        if len(positions) == 0:
            return False

        scale = 0.01 * 0.09349153968073806
        positions = positions[0]
        positions = np.array(list(map(lambda x: x * scale, positions)))

        if mapped:
            positions = self.get_mapped_position(awinda_reference, positions)
        return positions

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

    def send_ros_markers(self, positions, connections, frame_id="dev0", color=ColorRGBA(0.98, 0.30, 0.30, 1.00)):
        idx = 0
        marker_array = MarkerArray()
        conecction_points = set({})
        for connection in connections:
            conecction_points.update(connection)

        for i, position in enumerate(positions):
            # if i not in conecction_points:
            #     continue
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
            marker_array.markers.append(m_text)
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

    def extract_frame_data(self, frame):
        positions = frame.find("{http://www.xsens.com/mvn/mvnx}position")
        positions = positions.text.split()
        positions = list(map(lambda x: float(x) * self._scale, positions))
        joint_angle = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngle").text.split()
        joint_angle_xzy = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngleXZY").text.split()
        return (positions, joint_angle, joint_angle_xzy)

    def start_collecting(self, path: str, video_config):
        root = self.read_awinda_mvnx(path)
        point_names, connections = self.get_points_and_connections(root)

        # They are all the same. Therefore init this value one time
        if not self._awinda_connections:
            self._awinda_connections = connections

        self._mappings = MAPPING  # list(map(lambda x: (point_names.index(x[0]), x[1]), MAPPINGS_0))
        self.open_video(video_config["path_video"])

        point_posittions = dict(map(lambda x: (x, [0., 0., 0.]), point_names))
        frames = list(root.find("{http://www.xsens.com/mvn/mvnx}subject").find("{http://www.xsens.com/mvn/mvnx}frames"))

        self._frame_counter = 0
        self._frame_index = 0
        max_indices = len(frames)
        while self._frame_index < max_indices:

            frame = frames[self._frame_index]
            if self._frame_index <= video_config["frame_delay"]:
                self._frame_index += 1
                continue

            if self._frame_index > video_config["measure_end"]:
                break

            if frame.attrib["type"] != "normal":
                self._frame_index += 1
                continue

            if self._frame_index % 2 == 0:
                self._frame_index += 1
                continue

            if self._frame_index <= video_config["measure_start"]:
                self._frame_counter += self._skip_frames
                self._frame_index += self._skip_frames * 2
                continue

            positions, angles, _ = self.extract_frame_data(frame)

            if len(positions) % 3 != 0:
                print("[ERROR]: positions are not divisible by 3")
                break

            position_len = len(positions) // 3
            for i in range(position_len):
                point_posittions[point_names[i]] = positions[i * 3:(i + 1) * 3]

            frame = self.get_next_image(video_config)
            if USE_METRABS:
                awinda_pos = np.array(list(point_posittions.values()))
                metrabs_pos = self.get_metrabs(frame, awinda_pos)
                # print(metrabs_pos)
                if np.any(metrabs_pos):
                    self._snapshots.append({
                        "image": frame,
                        "metrabs_pos": metrabs_pos,
                        "awinda_pos": awinda_pos,
                        "angles": angles,
                    })

            self._frame_counter += self._skip_frames
            self._frame_index += self._skip_frames * 2

    def open_video(self, path: str):
        self._cap = cv2.VideoCapture(path)

    def get_next_image(self, viddeo_config):
        self._cap.set(cv2.CAP_PROP_POS_FRAMES, self._frame_counter)
        ret, frame = self._cap.read()
        if viddeo_config.get("rotate", False):
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return frame

    def display_collected_data(self):
        data_index = 0
        self._mappings = MAPPING
        while True:
            current_frame = self._snapshots[data_index]["image"]
            metrabs_pos = self._snapshots[data_index]["metrabs_pos"]
            awinda_pos = self._snapshots[data_index]["awinda_pos"]

            awinda_angles = self._snapshots[data_index]["angles"]

            angle_xsens = []
            for i in range(22):
                angle_xsens.append(awinda_angles[i * 3:(i + 1) * 3])

            # print(angle_xsens)
            print("rücken", angle_xsens[0])
            print("hüfte", angle_xsens[14])
            print()
            metrabs_pos = self.get_mapped_position(awinda_pos, metrabs_pos)

            cv2.imshow('img', current_frame)

            # indices = SKELETON_INDICES["smpl_24"]
            # metrabs_connections = list(map(lambda x: (indices[x[0]], indices[x[1]]), SMPL_CONNECTIONS))
            metrabs_connections = list(map(lambda x: (x[0], x[1]), TEST_CONNECTIONS))
            # print("len:", len(metrabs_connections))
            self.send_ros_markers(awinda_pos, self._awinda_connections)
            self.send_ros_markers(metrabs_pos, metrabs_connections, "dev1", ColorRGBA(0.30, 0.98, 0.30, 1.00))

            key = cv2.waitKey(500)
            if key == ord('a'):
                data_index = max(data_index - 1, 0)
            if key == ord('d'):
                data_index = min(data_index + 1, len(self._snapshots) - 1)
            if key == ord(' '):
                del self._snapshots[data_index]
                min(data_index, len(self._snapshots) - 1)
            if key == ord('b'):
                break

    def store(self, path):
        with open(path, 'wb') as f:
            pickle.dump(self._snapshots, f)

    def load(self, path):
        root = self.read_awinda_mvnx("data/awinda/gehen_links/gehen_links.mvnx")
        _, connections = self.get_points_and_connections(root)
        self._awinda_connections = connections
        self._snapshots = pickle.load(open(path, "rb"))


def main():
    generate_data = True
    converter = AwindaDataToRos(generate_data)

    def signal_handler(self, signal):
        print("############ SHUTDOWN ##################")
        nonlocal converter
        del converter
        sys.exit(0)

    time.sleep(2)
    signal.signal(signal.SIGINT, signal_handler)

    path = "./metrabs_data.pickle"
    if generate_data:
        for config in VIDEO_CONFIGS:
            print(config["path_mvnx"])
            converter.start_collecting(config["path_mvnx"], config)
        #print(sum(converter._scales) / len(converter._scales))
        converter.store(path)
    else:
        converter.load(path)
    converter.display_collected_data()


if __name__ == "__main__":
    main()
