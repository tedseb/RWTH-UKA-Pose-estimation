import pickle
import sys
import xml.etree.ElementTree as ET
import time
import signal
import copy
import subprocess
import cv2
import numpy as np
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
import tensorflow as tf
import rospy
import procrustes

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
    "measure_start": 380,
    "measure_end": 1700
}

VIDEO_CONFIG_STEHEN_VORN = {
    "path_video": "data/awinda/stehen_verschieden/Julia-006.mp4",
    "path_mvnx": "data/awinda/stehen_verschieden/Stehen_verschieden.mvnx",
    "frame_delay": 34,
    "measure_start": 200,
    "measure_end": 220
}

VIDEO_CONFIG = VIDEO_CONFIG_HAMPELMANN_VORN
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
    [120, 121],
    [121, 82],
    [121, 34],
    [82, 16],
    [34, 15],
    [16, 87],
    [15, 39]
]

SKELETON_INDICES = {
    "coco_19": [68, 72, 74, 56, 37, 64, 48, 52, 30, 104, 85, 112, 96, 100, 78, 41, 35, 89, 83],
    "smpl_24": [23, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
}


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
        self._angles = [0, 0, 0, 0]
        self._angles_count = 0
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

        # print(self._mappings)
        awinda_refs = np.array([awinda_reference[x[0]] for x in self._mappings])
        metrabs_refs = np.array([positions[x[1]] for x in self._mappings])
        # print(awinda_refs)
        # print(metrabs_refs)

        with TraceTime("Mapping"):
            d, Z, tform = procrustes.procrustes(awinda_refs, metrabs_refs, True, False)
        T = tform['rotation']
        b = tform['scale']
        c = tform['translation']
        # print(T)
        # print(b)
        # print(c)
        positions = b * positions @ T + c
        # print("###", b)
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
        # print("p1:", p1)
        # print("p2:", p2)
        # print("p3:", p3)
        # print("A:", a)
        # print("B:", b)
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
        # print("xy:", xy)
        # print("xz:", xz)
        # print("yz:", yz)
        # print("angle", angle)
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
            # print()
            # print("###### result #######")
            # print("# metrabs:", xy1, xz1, yz1, angle1)
            # print("# awinda:", xy2, xz2, yz2, angle2)
            # print(f"# {xy_diff:.1f}, {xz_diff:.1f}, {yz_diff:.1f}")
            # print(f"# {angle_diff:.1f}")
            # print("#####################")
            self._angles[0] += xy_diff
            self._angles[1] += xz_diff
            self._angles[2] += yz_diff
            self._angles[3] += angle_diff
            self._angles_count += 1

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

    def print_euclidean_result(self):
        pass
        # print()
        # print("###### Final Result #######")
        # print(f"# xy={xy_diff:.1f}°, xz={xz_diff:.1f}°, yz={yz_diff:.1f}°")
        # print(f"# a={angle_diff:.1f}°")
        # print("#####################")

    def print_angle_result(self):
        self._angles_count += 1
        xy_diff = self._angles[0] / self._angles_count
        xz_diff = self._angles[1] / self._angles_count
        yz_diff = self._angles[2] / self._angles_count
        angle_diff = self._angles[3] / self._angles_count

    def send_ros_markers(self, positions, connections, frame_id="dev0", color=ColorRGBA(0.98, 0.30, 0.30, 1.00)):
        idx = 0
        marker_array = MarkerArray()

        for i, position in enumerate(positions):
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
            m.lifetime = rospy.Duration(0.2)
            m_text = copy.deepcopy(m)
            m_text.id = idx
            idx += 1
            m_text.type = 9
            m_text.scale = Vector3(0.052, 0.052, 0.052)
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
            m.lifetime = rospy.Duration(0.2)
            marker_array.markers.append(m)

        if frame_id == "dev0":
            self._skeleton_pub0.publish(marker_array)
        else:
            self._skeleton_pub1.publish(marker_array)

    def start(self, path: str):
        root = self.read_awinda_mvnx(path)
        point_names, connections = self.get_points_and_connections(root)
        self._mappings = MAPPING  # list(map(lambda x: (point_names.index(x[0]), x[1]), MAPPINGS_0))

        point_posittions = dict(map(lambda x: (x, [0., 0., 0.]), point_names))
        frames = root.find("{http://www.xsens.com/mvn/mvnx}subject").find("{http://www.xsens.com/mvn/mvnx}frames")
        start_time_ms = time.time() * 1000

        for frame_counter, frame in enumerate(frames):
            if frame_counter <= VIDEO_CONFIG["frame_delay"]:
                continue

            if frame.attrib["type"] != "normal":
                continue

            if frame_counter % 2 == 1:
                continue

            positions = frame.find("{http://www.xsens.com/mvn/mvnx}position")
            positions = positions.text.split()
            positions = list(map(lambda x: float(x) * self._scale, positions))
            joint_angle = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngle").text.split()
            joint_angle_xzy = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngleXZY").text.split()
            # joint_angle_ergo = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngleErgo").text.split()
            # joint_angle_ergo_xzy = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngleErgoXZY").text.split()

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
            # print(point_posittions)
            # print(point_posittions)

            self.send_video()
            if USE_METRABS:
                awinda_pos = np.array(list(point_posittions.values()))
                metrabs_pos, tform = self.send_metrabs(self._cur_frame, awinda_pos)
                if np.any(metrabs_pos):
                    if VIDEO_CONFIG["measure_start"] <= frame_counter <= VIDEO_CONFIG["measure_end"]:
                        # print(f"### {frame_counter} ###")
                        self.compute_angle_distance(point_posittions, metrabs_pos)
                        # self.compute_euclidean_distance(awinda_pos, metrabs_pos)
                        # time.sleep(0.2)

                if frame_counter == VIDEO_CONFIG["measure_end"]:
                    self.print_angle_result()
                    # self.print_euclidean_result()

                if frame_counter == 378:
                    print("# save #")
                    with open('awinda_data.pickle', 'wb') as f:
                        data = {
                            "awinda_pos": list(point_posittions.values()),
                            "connections": connections,
                            "image": self._cur_frame,
                            "transformation": tform,
                            "frame_id": frame_counter,
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

            # if frame_counter > 500:
            #     break

    def open_video(self, path: str):
        self._cap = cv2.VideoCapture(path)

    def send_video(self):
        ret, self._cur_frame = self._cap.read()

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
