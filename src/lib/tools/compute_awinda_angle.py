import pickle
import sys
import xml.etree.ElementTree as ET
import time
import signal
import copy
import subprocess
import cv2
import numpy as np
import procrustes
from awinda.angle_computation import (
    calculate_ergo_pelvis_rotation,
    calculate_knee_rotation,
    calculate_pelvis_rotation,
    calculate_ergo_upper_arm_rotation,
    calculate_ergo_upper_arm_rotation2,
    calculate_upper_arm_rotation
)


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)) * (180 / np.pi)


def check_rotation_matrix( M, orth_thresh=1e-5, det_thresh=1e-5 ):
    if np.linalg.norm( M[:3,:3].T@M[:3,:3] - np.eye(3) ) > orth_thresh:
        print(M[:3,:3].T@M[:3,:3])
        raise ValueError('Input matrix is not a pure rotation 1')
    if np.abs(np.linalg.det(M[:3,:3])-1.0) > det_thresh:
        print(np.abs(np.linalg.det(M[:3,:3])-1.0))
        raise ValueError('Input matrix is not a pure rotation 2')


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


class AwindaParser:
    def __init__(self):
        self._angles = [0, 0, 0, 0]
        self._angles_count = 0

    def __del__(self):
        pass

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


    def start(self, path: str):
        np.set_printoptions(suppress=True)
        np.set_printoptions(precision=4)

        root = self.read_awinda_mvnx(path)
        point_names, connections = self.get_points_and_connections(root)

        point_posittions = dict(map(lambda x: (x, [0., 0., 0.]), point_names))
        frames = root.find("{http://www.xsens.com/mvn/mvnx}subject").find("{http://www.xsens.com/mvn/mvnx}frames")
        start_time_ms = time.time() * 1000

        diff_list = []
        for frame_counter, frame in enumerate(frames):
            if frame_counter <= 0:
                continue

            if frame.attrib["type"] != "normal":
                continue

            if frame_counter % 2 == 1:
                continue

            positions = frame.find("{http://www.xsens.com/mvn/mvnx}position")
            positions = positions.text.split()
            positions = list(map(lambda x: float(x), positions))
            joint_angle_zxy = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngle").text.split()
            #joint_angle_xzy = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngleXZY").text.split()
            joint_angle_ergo = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngleErgo").text.split()
            # joint_angle_ergo_xzy = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngleErgoXZY").text.split()

            angle_zxy_xsens = []
            angle_xzy_xsens = []
            angle_ergo_zxy_xsens = []
            for i in range(22):
                angle_zxy_xsens.append(joint_angle_zxy[i * 3:(i + 1) * 3])
                # angle_xzy_xsens.append(joint_angle_xzy[i * 3:(i + 1) * 3])

            for i in range(6):
                angle_ergo_zxy_xsens.append(joint_angle_ergo[i * 3:(i + 1) * 3])

            if len(positions) % 3 != 0:
                print("[ERROR]: positions are not divisible by 3")
                break

            position_len = len(positions) // 3
            for i in range(position_len):
                point_posittions[point_names[i]] = positions[i * 3:(i + 1) * 3]

            frame_time_stamp_ms = float(frame.attrib["time"])
            time_now_ms = time.time() * 1000
            time_diff_ms = time_now_ms - start_time_ms
            wait_time_s = max(frame_time_stamp_ms - time_diff_ms, 0) / 1000.

            # Knee
            xzy_rot_knee = calculate_knee_rotation(
                np.array(point_posittions["RightUpperLeg"]),
                np.array(point_posittions["RightLowerLeg"]),
                np.array(point_posittions["RightFoot"]),
                np.array(point_posittions["RightToe"])
            )
            x_sense_knee = np.array(list(map(float, angle_zxy_xsens[15])))
            ours_knee = xzy_rot_knee[0] * (180 / np.pi)
            diff_knee = np.absolute(x_sense_knee - ours_knee)
            # diff_list.append(diff_knee)

            # Pelvis
            xzy_rot_pelvis = calculate_ergo_pelvis_rotation(
                np.array(point_posittions["RightUpperLeg"]),
                np.array(point_posittions["Pelvis"]),
                np.array(point_posittions["L5"]),
            )
            x_sense_pelvis = np.array(list(map(float, angle_ergo_zxy_xsens[4])))
            ours_pelvis = xzy_rot_pelvis[0] * (180 / np.pi)
            ours_pelvis[2] += 6.3545
            diff_pelvis = np.absolute(x_sense_pelvis - ours_pelvis)
            # print(x_sense_pelvis, ours_pelvis, diff_pelvis)
            # diff_list.append(diff_pelvis)

            # Pelvis new
            xzy_rot_pelvis = calculate_pelvis_rotation(
                np.array(point_posittions["RightLowerLeg"]),
                np.array(point_posittions["RightUpperLeg"]),
                np.array(point_posittions["Pelvis"]),
                np.array(point_posittions["L5"])
            )
            x_sense_pelvis = np.array(list(map(float, angle_zxy_xsens[0])))
            x_sense_hips = np.array(list(map(float, angle_zxy_xsens[14])))
            ours_pelvis = xzy_rot_pelvis[0] * (180 / np.pi)
            diff_pelvis = np.absolute(x_sense_pelvis - ours_pelvis)
            # diff_list.append(diff_pelvis)

            # right upper arm ergo
            xzy_rot_upper_arm = calculate_ergo_upper_arm_rotation2(
                np.array(point_posittions["T8"]),
                np.array(point_posittions["Neck"]),
                np.array(point_posittions["RightShoulder"]),
                np.array(point_posittions["RightUpperArm"]),
                np.array(point_posittions["RightForeArm"]),
            )
            x_sense_left_shoulder_ergo = np.array(list(map(float, angle_ergo_zxy_xsens[2])))
            x_sense_left_shoulder = np.array(list(map(float, angle_zxy_xsens[7])))
            # x_sense_hips = np.array(list(map(float, angle_zxy_xsens[14])))
            ours_upper_arm_ergo = xzy_rot_upper_arm[0] * (180 / np.pi)
            # diff_pelvis = np.absolute(x_sense_pelvis - ours_pelvis)
            # diff_list.append(diff_pelvis)

            # right upper arm
            xzy_rot_upper_arm = calculate_upper_arm_rotation(
                np.array(point_posittions["T8"]),
                np.array(point_posittions["Neck"]),
                np.array(point_posittions["RightShoulder"]),
                np.array(point_posittions["RightUpperArm"]),
                np.array(point_posittions["RightForeArm"]),
                np.array(point_posittions["RightHand"])
            )
            ours_upper_arm = xzy_rot_upper_arm[0] * (180 / np.pi)
            diff_upper_arm = np.absolute(x_sense_left_shoulder_ergo - ours_upper_arm_ergo)
            diff_list.append(diff_upper_arm)
            print(x_sense_left_shoulder_ergo, ours_upper_arm_ergo, diff_upper_arm)
            # print("frame:", frame_counter, "knee:", x_sense_knee, ", hips:", x_sense_hips, " pelvis:", x_sense_pelvis, " ours:", ours_pelvis, " diff", diff_pelvis) # ", ours:", ours, " diff:", diff)
            time.sleep(wait_time_s)
            if frame_counter > 500:
                break

        diff_list = np.array(diff_list)
        print("Average diff: ", np.average(diff_list, 0))
        print("Median diff: ", np.median(diff_list, 0))

    def open_video(self, path: str):
        self._cap = cv2.VideoCapture(path)

def main():
    converter = AwindaParser()

    def signal_handler(self, signal):
        print("############ SHUTDOWN ##################")
        nonlocal converter
        del converter
        sys.exit(0)

    time.sleep(2)
    signal.signal(signal.SIGINT, signal_handler)
    converter.start("data/awinda/kniebeugen_vorne/Kniebeuge_vorne.mvnx")


if __name__ == "__main__":
    main()
