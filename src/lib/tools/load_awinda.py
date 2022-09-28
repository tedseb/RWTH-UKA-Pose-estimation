import sys
import xml.etree.ElementTree as ET
import time
import signal
import subprocess
import cv2
import numpy as np
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
import tensorflow as tf
import rospy

PATH_MVNX = "data/awinda/Test-007#Hannah2.mvnx"
# PATH_VIDEO = "data/awinda/Test-007_drehen#Hannah2.mp4"
PATH_VIDEO = "data/double_squats_02.mp4"
METRABS_PATH = "/home/trainerai/trainerai-core/src/AI/metrabs/models/metrabs_eff2m_y4"
USE_METRABS = True
COMPARE_ANGLES = [
    ["LeftUpperLeg", "LeftLowerLeg", "LeftFoot"],
    ["RightUpperLeg", "RightLowerLeg", "RightFoot"]
]
SMPL_CONNECTIONS = [
    (1, 4), (1, 0), (2, 5), (2, 0), (3, 6), (3, 0), (4, 7), (5, 8), (6, 9), (7, 10), (8, 11), (9, 12), (12, 13),
    (12, 14), (12, 15), (13, 16), (14, 17), (16, 18), (17, 19), (18, 20), (19, 21), (20, 22), (21, 23)
]

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
        print(f"Time {self._name}: {time_elapsed}ms")


class AwindaDataToRos:
    def __init__(self):
        rospy.init_node('awinda_data', anonymous=False)
        transform_path = "/home/trainerai/trainerai-core/src/station_manager/launch/static_transform.launch"
        self._transform = subprocess.Popen(["roslaunch", transform_path, "dev:=0"])
        self._skeleton_pub = rospy.Publisher("/visualization/skeleton_0", MarkerArray, queue_size=5)
        self._image_pub = rospy.Publisher("/image/channel_0_yolo", Image, queue_size=5)
        self._skeleton_scale = Vector3(0.09, 0.09, 0.09)
        self._connection_scale = Vector3(0.03, 0.03, 0.03)
        self._scale = 3
        self._cap = None
        self._model = None
        if USE_METRABS:
            self.init_metrabs()
        self.open_video(PATH_VIDEO)

    def __del__(self):
        self._transform.terminate()
        try:
            self._transform.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.kill(self._transform.pid)
        del self._transform
        rospy.signal_shutdown("End")

    def init_metrabs(self):
        self._model = tf.saved_model.load(METRABS_PATH)

    def send_metrabs(self, image):
        if self._model is None:
            return

        with TraceTime("Metrabs"):
            pred = self._model.detect_poses(
                image, skeleton='smpl_24', default_fov_degrees=55, detector_threshold=0.5)

        positions = pred['poses3d'].numpy()[0]
        print(positions)
        self.send_ros_markers(positions, SMPL_CONNECTIONS, "dev1", ColorRGBA(0.98, 0.30, 0.30, 1.00))

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

    def compute_angles(self, point_positions):
        for angle in COMPARE_ANGLES:
            print(f" -- {angle} -- ")
            p1 = np.array(point_positions[angle[0]])
            p2 = np.array(point_positions[angle[1]])
            p3 = np.array(point_positions[angle[2]])
            a = p1 - p2
            b = p3 - p2
            print("p1:", p1)
            print("p2:", p2)
            print("p3:", p3)
            print("A:", a)
            print("B:", b)
            xy = np.arctan2(a[0], a[1]) - np.arctan2(b[0], b[1])
            xz = np.arctan2(a[0], a[2]) - np.arctan2(b[0], b[2])
            yz = np.arctan2(a[1], a[2]) - np.arctan2(b[1], b[2])
            xy = np.absolute(xy) * (180 / np.pi)
            xz = np.absolute(xz) * (180 / np.pi)
            yz = np.absolute(yz) * (180 / np.pi)
            print("xy:", xy)
            print("xz:", xz)
            print("yz:", yz)
            print("angle", angle_between(a, b))

    def send_ros_markers(self, positions, connections, frame_id="dev0", color=ColorRGBA(0.98, 0.30, 0.30, 1.00)):
        idx = 0
        marker_array = MarkerArray()

        for position in positions:
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

        self._skeleton_pub.publish(marker_array)

    def start(self, path: str):
        root = self.read_awinda_mvnx(path)
        point_names, connections = self.get_points_and_connections(root)

        point_posittions = dict(map(lambda x: (x, [0., 0., 0.]), point_names))
        frames = root.find("{http://www.xsens.com/mvn/mvnx}subject").find("{http://www.xsens.com/mvn/mvnx}frames")
        start_time_ms = time.time() * 1000

        for frame_counter, frame in enumerate(frames):
            if frame.attrib["type"] != "normal":
                continue

            print(f"### {frame_counter} ###")
            positions = frame.find("{http://www.xsens.com/mvn/mvnx}position")
            positions = positions.text.split()
            positions = list(map(lambda x: float(x) * self._scale, positions))

            if len(positions) % 3 != 0:
                print("[ERROR]: positions are not divisible by 3")
                break

            position_len = len(positions) // 3
            for i in range(position_len):
                point_posittions[point_names[i]] = positions[i * 3:(i + 1) * 3]

            self.compute_angles(point_posittions)
            self.send_video()
            if USE_METRABS:
                self.send_metrabs(self._cur_frame)
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
    converter.start(PATH_MVNX)


if __name__ == "__main__":
    main()

import numpy as np
import procrustes

def main():
    x = np.array([[0, 0, 1], [0, 1, 1]])
    y = np.array([[2, 0, 0], [2, 2, 0]])

    d, Z, tform = procrustes.procrustes(x, y, True, False)
    T = tform['rotation']
    b = tform['scale']
    c = tform['translation']
    print(T)
    print(b)
    print(c)
    y_new = b * y @ T + c
    print("###### should be #####")
    print(x)
    print("######   is now  #####")
    print(y_new)


def main2():
    p1 = np.array([1, 1, 1])
    p2 = np.array([0, 0, 0])
    p3 = np.array([1, 0, 1])
    a = p1 - p2
    b = p3 - p2
    print(a)
    print(b)
    xy = np.arctan2(a[0], a[1]) - np.arctan2(b[0], b[1])
    xz = np.arctan2(a[0], a[2]) - np.arctan2(b[0], b[2])
    yz = np.arctan2(a[1], a[2]) - np.arctan2(b[1], b[2])
    xy = np.absolute(xy) * (180 / np.pi)
    xz = np.absolute(xz) * (180 / np.pi)
    yz = np.absolute(yz) * (180 / np.pi)

    print(xy)
    print(xz)
    print(yz)


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)) * (180 / np.pi)


def main3():
    a = np.array([0.31, -1.64, 0.42])
    b = np.array([-0.39, -0.04, -1.13])
    print(angle_between(a, b))


if __name__ == "__main__":
    main3()
