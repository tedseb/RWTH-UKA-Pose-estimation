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


PATH_IMAGE = "/home/trainerai/trainerai-core/src/lib/lib_data/show_skeleton.jpg"
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
SKELETON = "smpl_24"


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
        self._skeleton_pub0 = rospy.Publisher("/visualization/skeleton_0", MarkerArray, queue_size=5)
        self._image_pub = rospy.Publisher("/image/channel_0_yolo", Image, queue_size=5)
        self._skeleton_scale = Vector3(0.2, 0.2, 0.2)
        self._connection_scale = Vector3(0.09, 0.09, 0.09)
        self._scale = 2
        self._image = cv2.imread(PATH_IMAGE)
        self._model = None

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

    def send_metrabs(self):
        if self._model is None:
            return False

        pred = self._model.detect_poses(
            self._image, skeleton='', default_fov_degrees=55, detector_threshold=0.5)

        positions = pred['poses3d'].numpy()
        if len(positions) == 0:
            return False

        scale = 0.01
        positions = positions[0]
        pelvis = positions[0]
        positions = np.array(list(map(lambda x: rotate_dim_x((x - pelvis) * scale, -90), positions)))

        self.send_ros_markers(positions, CONNECTIONS[SKELETON])
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
            m_text.scale = Vector3(0.5, 0.5, 0.5)
            m_text.text = f"\n   {i}"
            marker_array.markers.append(m_text)
            marker_array.markers.append(m)

        # for connection in connections:
        #     m = Marker()
        #     m.header.stamp = rospy.Time.now()
        #     m.header.frame_id = frame_id
        #     m.id = idx
        #     idx += 1
        #     m.ns = ''
        #     m.color = color
        #     m.scale = self._connection_scale
        #     m.points = [Point(*positions[connection[0]]), Point(*positions[connection[1]])]
        #     m.type = 4
        #     m.action = 0
        #     m.lifetime = rospy.Duration(2)
        #     marker_array.markers.append(m)

        if frame_id == "dev0":
            # print("send metrabs")
            self._skeleton_pub0.publish(marker_array)
        else:
            self._skeleton_pub1.publish(marker_array)

    def start(self):
        while True:
            self.send_metrabs()
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
