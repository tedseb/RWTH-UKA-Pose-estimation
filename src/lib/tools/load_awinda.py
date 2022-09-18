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
import rospy

PATH_MVNX = "data/awinda/Test-007#Hannah2.mvnx"
PATH_VIDEO = "data/awinda/Test-007_drehen#Hannah2.mp4"


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
        self.open_video(PATH_VIDEO)

    def __del__(self):
        self._transform.terminate()
        try:
            self._transform.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.kill(self._transform.pid)
        del self._transform
        rospy.signal_shutdown("End")

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
                connection.append(connector.text.split("/")[0])
            connections.append(connection)
        return (points, connections)

    def send_ros_markers(self, positions, connections):
        idx = 0
        marker_array = MarkerArray()

        for position in positions.values():
            m = Marker()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "dev0"
            m.id = idx
            idx += 1
            m.ns = ''
            m.color = ColorRGBA(0.98, 0.30, 0.30, 1.00)
            m.scale = self._skeleton_scale
            m.pose.position.x, m.pose.position.y, m.pose.position.z = position
            m.type = 2
            m.action = 0
            m.lifetime = rospy.Duration(0.2)
            marker_array.markers.append(m)

        for connection in connections:
            m = Marker()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "dev0"
            m.id = idx
            idx += 1
            m.ns = ''
            m.color = ColorRGBA(0.98, 0.30, 0.30, 1.00)
            m.scale = self._connection_scale
            m.points = [Point(*positions[(connection[0])]), Point(*positions[connection[1]])]
            m.type = 4
            m.action = 0
            m.lifetime = rospy.Duration(0.2)
            marker_array.markers.append(m)

        self._skeleton_pub.publish(marker_array)

    def start(self, path: str):
        root = self.read_awinda_mvnx(path)
        points, connections = self.get_points_and_connections(root)

        point_posittions = dict(map(lambda x: (x, [0., 0., 0.]), points))
        frames = root.find("{http://www.xsens.com/mvn/mvnx}subject").find("{http://www.xsens.com/mvn/mvnx}frames")
        start_time_ms = time.time() * 1000

        for frame_counter, frame in enumerate(frames):
            if frame.attrib["type"] != "normal":
                continue

            positions = frame.find("{http://www.xsens.com/mvn/mvnx}position")
            positions = positions.text.split()
            positions = list(map(lambda x: float(x) * self._scale, positions))

            if len(positions) % 3 != 0:
                print("[ERROR]: positions are not divisible by 3")
                break

            position_len = len(positions) // 3
            for i in range(position_len):
                point_posittions[points[i]] = positions[i * 3:(i + 1) * 3]

            self.send_ros_markers(point_posittions, connections)
            self.send_video()

            print(frame.attrib["time"])

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
        img.step = self._cur_frame.shape[-1]*self._cur_frame.shape[0]
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
