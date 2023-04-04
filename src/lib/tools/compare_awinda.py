import sys
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
import math
import pickle
import procrustes
import json

PICKLE_PATH = "awinda_data.pickle"
METRABS_PATH = "/home/trainerai/trainerai-core/src/AI/metrabs/models/metrabs_rn34_y4"
# METRABS_PATH = "/home/trainerai/trainerai-core/src/AI/metrabs/models/metrabs_eff2m_y4"
CONNECTIONS = {
    "smpl_24": [
        (1, 4), (1, 0), (2, 5), (2, 0), (3, 6), (3, 0), (4, 7), (5, 8), (6, 9), (7, 10), (8, 11), (9, 12),
        (12, 13), (12, 14), (12, 15), (13, 16), (14, 17), (16, 18), (17, 19), (18, 20), (19, 21), (20, 22), (21, 23)
    ],
    "smpl+head_30": [
        (1, 4), (1, 0), (2, 5), (2, 0), (3, 6), (3, 0), (4, 7), (5, 8), (6, 9), (7, 10), (8, 11), (9, 12),
        (12, 13), (12, 14), (12, 15), (13, 16), (14, 17), (16, 18), (17, 19), (18, 20), (19, 21), (20, 22), (21, 23)
    ],
    "h36m_25": [
        (12, 13), (12, 11), (7, 8), (7, 6), (15, 14), (15, 16), (18, 16), (8, 9), (5, 6), (14, 11), (17, 16),
        (11, 19), (11, 10), (2, 3), (2, 1), (20, 19), (20, 21), (23, 21), (3, 4), (0, 1), (22, 21)
    ],
    "h36m_17": [
        (9, 10), (9, 8), (6, 5), (12, 11), (12, 13), (4, 5), (4, 0), (11, 8), (8, 14), (8, 7), (0, 1), (0, 7), (3, 2),
        (15, 14), (15, 16), (1, 2)
    ],
    "mpi_inf_3dhp_17": [
        (16, 0), (16, 1), (13, 12), (6, 5), (6, 7), (11, 12), (11, 14), (14, 8), (14, 15), (10, 9), (3, 2), (3, 4),
        (8, 9), (2, 1), (1, 15), (5, 1)
    ],
    "coco_19": [
        (8, 7), (16, 15), (4, 3), (4, 5), (15, 1), (6, 7), (6, 2), (3, 0), (0, 1), (0, 2), (0, 9), (1, 17), (2, 12),
        (14, 13), (18, 17), (10, 9), (10, 11), (12, 13)
    ]
}

COMPUTE_SKELETONS = True
SKELETON = "smpl_24"
SKELETON_INDICES = {
    "coco_19": [68, 72, 74, 56, 37, 64, 48, 52, 30, 104, 85, 112, 96, 100, 78, 41, 35, 89, 83],
    "smpl_24": [23, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
}

# MAPPINGS = [
#     (7, 82),
#     (11, 34),
#     (15, 97),
#     (0, 75),
#     (19, 49),
# ]

MAPPINGS = [
    (8, 16),
    (12, 15),
    # (0, 23),
    (15, 99),
    (19, 51),
    (16, 4),
    (20, 3),
    (14, 43),
    (10, 91)
]


def rotate_dim_x(coordinates, degree):
    radians = math.radians(degree)
    rotated_coordinates = [0., 0., 0.]
    rotated_coordinates[0] = coordinates[0]
    rotated_coordinates[1] = coordinates[1] * math.cos(radians) - coordinates[2] * math.sin(radians)
    rotated_coordinates[2] = coordinates[1] * math.sin(radians) + coordinates[2] * math.cos(radians)
    return rotated_coordinates


class ShowSkeleton:
    def __init__(self):
        rospy.init_node('show_skeleton', anonymous=False)

        transform_path = "/home/trainerai/trainerai-core/src/station_manager/launch/static_transform.launch"
        self._transform1 = subprocess.Popen(["roslaunch", transform_path, "dev:=0"])
        self._transform2 = subprocess.Popen(["roslaunch", transform_path, "dev:=1"])

        self._skeleton_pub0 = rospy.Publisher("/visualization/skeleton_0", MarkerArray, queue_size=5)
        self._skeleton_pub1 = rospy.Publisher("/visualization/skeleton_1", MarkerArray, queue_size=5)

        self._image_pub = rospy.Publisher("/image/channel_0_yolo", Image, queue_size=5)
        self._skeleton_scale = Vector3(0.02, 0.02, 0.02)
        self._connection_scale = Vector3(0.01, 0.01, 0.01)
        self._scale = 2
        self._model = None

        image_info = pickle.load(open(PICKLE_PATH, "rb"))
        self._image = image_info["image"]
        self._awinda_skeleton = image_info["awinda_pos"]
        self._connections = image_info["connections"]
        self._transformation = image_info["transformation"]
        self._frame_id = image_info["frame_id"]
        self._angles = image_info["angles"]
        self._angles_xzy = image_info["angles_xzy"]
        angle_xsens = []
        angle_xzy_xsens = []
        for i in range(22):
            angle_xsens.append(self._angles[i * 3:(i + 1) * 3])
            angle_xzy_xsens.append(self._angles_xzy[i * 3:(i + 1) * 3])
        print("angles", angle_xsens[19])
        print("angles_xzy", angle_xzy_xsens[19])
        self.init_metrabs()

    def __del__(self):
        self._transform1.terminate()
        try:
            self._transform1.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.kill(self._transform1.pid)
        del self._transform1

        rospy.signal_shutdown("End")

    def init_metrabs(self):
        self._model = tf.saved_model.load(METRABS_PATH)

    def send_metrabs(self, awinda_reference):
        if self._model is None:
            return False

        pred = self._model.detect_poses(
            self._image, skeleton='', default_fov_degrees=55, detector_threshold=0.5)

        positions = pred['poses3d'].numpy()
        if len(positions) == 0:
            return False

        scale = 0.01
        positions = positions[0]
        # pelvis = positions[0]
        positions = np.array(list(map(lambda x: x * scale, positions)))

        # mappings = MAPPINGS
        # connections = CONNECTIONS[SKELETON]
        f = open('mappings.json')
        data = json.load(f)
        mappings = data["mappings"]
        connections = data["connections"]
        # print(mappings, connections)

        awinda_refs = np.array([awinda_reference[x[0]] for x in mappings])
        metrabs_refs = np.array([positions[x[1]] for x in mappings])
        # print(awinda_refs)
        # print(metrabs_refs)

        # with TraceTime("Mapping"):
        d, Z, tform = procrustes.procrustes(awinda_refs, metrabs_refs, True, False)
        # T = self._transformation['rotation']
        # b = self._transformation['scale']
        # c = self._transformation['translation']
        T = tform['rotation']
        b = tform['scale']
        c = tform['translation']
        # print("###", b)
        positions = b * positions @ T + c

        if COMPUTE_SKELETONS:
            indices = SKELETON_INDICES[SKELETON]
            connections = list(map(lambda x: (x[0], x[1]), connections))
            # print(connections)
            # connections = list(map(lambda x: (indices[x[0]], indices[x[1]]), connections))

            self.send_ros_markers(positions, connections, "dev1", ColorRGBA(0.30, 0.98, 0.30, 1.00))
        else:
            self.send_ros_markers(positions, [], "dev1", ColorRGBA(0.30, 0.98, 0.30, 1.00))
        return positions

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
            m.lifetime = rospy.Duration(2)
            m_text = copy.deepcopy(m)
            m_text.id = idx
            idx += 1
            m_text.type = 9
            m_text.scale = Vector3(0.02, 0.02, 0.02)
            m_text.text = f"\n   {i}"
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
            m.lifetime = rospy.Duration(2)
            marker_array.markers.append(m)

        if frame_id == "dev0":
            # print("send metrabs")
            self._skeleton_pub0.publish(marker_array)
        else:
            self._skeleton_pub1.publish(marker_array)

    def send_video(self):
        img = Image()
        img.header.stamp = rospy.Time.now()
        img.header.frame_id = "dev0"
        img.encoding = "bgr8"
        img.data = np.array(self._image, dtype=np.uint8).tobytes()
        img.height, img.width = self._image.shape[:-1]
        img.step = self._image.shape[-1] * self._image.shape[0]
        self._image_pub.publish(img)

    def start(self):
        while True:
            self.send_ros_markers(self._awinda_skeleton, self._connections)
            self.send_metrabs(self._awinda_skeleton)
            self.send_video()
            time.sleep(1)


def main():
    visualizer = ShowSkeleton()

    def signal_handler(self, signal):
        print("############ SHUTDOWN ##################")
        nonlocal visualizer
        del visualizer
        sys.exit(0)

    time.sleep(2)
    signal.signal(signal.SIGINT, signal_handler)
    visualizer.start()


if __name__ == "__main__":
    main()
