import sys
import xml.etree.ElementTree as ET
import time
import signal
import subprocess
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
import rospy

PATH = "data/awinda/Test-007#Hannah2.mvnx"


class AwindaDataToRos:
    def __init__(self):
        rospy.init_node('awinda_data', anonymous=False)
        transform_path = "/home/trainerai/trainerai-core/src/station_manager/launch/static_transform.launch"
        self._transform = subprocess.Popen(["roslaunch", transform_path, "dev:=0"])
        self._pub = rospy.Publisher("/visualization/skeleton_0", MarkerArray, queue_size=100)
        self._skeleton_scale = Vector3(0.03, 0.03, 0.03)
        self._connection_scale = Vector3(0.01, 0.01, 0.01)

    def __del__(self):
        self._transform.terminate()
        try:
            self._transform.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.kill(self._transform.pid)
        del self._transform

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

        self._pub.publish(marker_array)

    def start(self, path):
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
            positions = list(map(lambda x: float(x), positions))

            if len(positions) % 3 != 0:
                print("[ERROR]: positions are not divisible by 3")
                break

            position_len = len(positions) // 3
            for i in range(position_len):
                point_posittions[points[i]] = positions[i * 3:(i + 1) * 3]

            self.send_ros_markers(point_posittions, connections)

            print(frame.attrib["time"])

            frame_time_stamp_ms = float(frame.attrib["time"])
            time_now_ms = time.time() * 1000
            time_diff_ms = time_now_ms - start_time_ms
            wait_time_s = max(frame_time_stamp_ms - time_diff_ms, 0) / 1000.
            # print("frame:", frame_time_stamp_ms, "stamp:", time_diff_ms, "wait:", wait_time_s)
            time.sleep(wait_time_s)

            # if frame_counter > 500:
            #     break


def main():
    converter = AwindaDataToRos()

    def signal_handler(self, signal):
        print("############ SHUTDOWN ##################")
        nonlocal converter
        del converter
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    converter.start(PATH)


if __name__ == "__main__":
    main()
