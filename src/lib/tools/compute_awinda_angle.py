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


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)) * (180 / np.pi)


def check_rotation_matrix( M, orth_thresh=1e-5, det_thresh=1e-5 ):
    if np.linalg.norm( M[:3,:3].T@M[:3,:3] - np.eye(3) ) > orth_thresh:
        print(M[:3,:3].T@M[:3,:3])
        raise ValueError('Input matrix is not a pure rotation')
    if np.abs(np.linalg.det(M[:3,:3])-1.0) > det_thresh:
        raise ValueError('Input matrix is not a pure rotation')


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

    @staticmethod
    def inverse_rotation_zxy( M, thresh=0.9999999 ):
        check_rotation_matrix( M )
        if np.abs(M[1,2]) > thresh:        
            sx = np.sign(-M[1,2])
            x0 = sx*np.pi/2

            # arbitrarily set z=0
            y0 = 0 # so sy=0, cy=1

            # compute x = arctan2( M[0,1]/sy, M[02]/sy )
            z0 = np.arctan2( M[2,0]/sx, M[2,1]/sx )
            return np.array((x0,y0,z0)), np.array((x0,y0,z0))
        else:
            x0 = np.arcsin( -M[1,2] )
            x1 = np.pi - x0
            c0 = np.cos(x0)
            c1 = np.cos(x1)

            y0 = np.arctan2( M[0,2]/c0, M[2,2]/c0 )
            y1 = np.arctan2( M[0,2]/c1, M[2,2]/c1 )

            z0 = np.arctan2( M[1,0]/c0, M[1,1]/c0 )
            z1 = np.arctan2( M[1,0]/c1, M[1,1]/c1 )
            return np.array((x0,y0,z0)), np.array((x1,y1,z1))


    @staticmethod
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
        return AwindaParser.inverse_rotation_zxy(basis_rotation)


    def start(self, path: str):
        np.set_printoptions(suppress=True)
        np.set_printoptions(precision=3)

        root = self.read_awinda_mvnx(path)
        point_names, connections = self.get_points_and_connections(root)

        point_posittions = dict(map(lambda x: (x, [0., 0., 0.]), point_names))
        frames = root.find("{http://www.xsens.com/mvn/mvnx}subject").find("{http://www.xsens.com/mvn/mvnx}frames")
        start_time_ms = time.time() * 1000

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
            joint_angle_xzy = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngleXZY").text.split()
            # joint_angle_ergo = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngleErgo").text.split()
            # joint_angle_ergo_xzy = frame.find("{http://www.xsens.com/mvn/mvnx}jointAngleErgoXZY").text.split()

            angle_zxy_xsens = []
            angle_xzy_xsens = []
            for i in range(22):
                angle_zxy_xsens.append(joint_angle_zxy[i * 3:(i + 1) * 3])
                angle_xzy_xsens.append(joint_angle_xzy[i * 3:(i + 1) * 3])

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
            xzy_rot = self.calculate_knee_rotation(
                np.array(point_posittions["RightUpperLeg"]), 
                np.array(point_posittions["RightLowerLeg"]), 
                np.array(point_posittions["RightFoot"]), 
                np.array(point_posittions["RightToe"])
            )
            print("frame:", frame_counter, ", xsens:", angle_zxy_xsens[15], ", ours:", xzy_rot[0] * (180 / np.pi))
            time.sleep(wait_time_s)

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
