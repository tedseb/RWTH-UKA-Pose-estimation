#!/usr/bin/python3
import rospy
import cv2
from sensor_msgs.msg import Image
import numpy as np
import pafy
import argparse
import pylint
import sys
from enum import Enum
# from utils.imutils import crop_bboxInfo, process_image_bbox, process_image_keypoints, bbox_from_keypoints ToDo: Do cropping here.

class VideoMode(Enum):
    INVALID = 0
    YOUTUBE = 1
    WEB_CAM = 2
    IP_CAM = 3
    DISK_VIDEO = 4

def returnCameraIndices():
    index = 0
    arr = []
    i = 10
    while i > 0:
        cap = cv2.VideoCapture(index)
        if cap.read()[0]:
            arr.append(index)
            cap.release()
        index += 1
        i -= 1
    return arr

class CameraNode():
    def __init__(self, verbose = False, dev_id = 0, check_cameras = False, camera_mode = VideoMode.INVALID, video_info = None):
        self._cap = None
        self._verbose = verbose
        self._camera_mode = camera_mode
        self._dev_id = "dev" + str(dev_id)
        self._youtube_mode = False

        rospy.init_node('camera', anonymous=True)
        self._pub = rospy.Publisher('image', Image, queue_size=2)

        if self._camera_mode is VideoMode.INVALID:
            raise RuntimeError("Invalid video mode")
        elif self._camera_mode is VideoMode.YOUTUBE:
            url = f"https://www.youtube.com/watch?v={video_info}"
            self.set_youtube_stream(url)
            self._youtube_mode = True
        elif self._camera_mode is VideoMode.WEB_CAM:
            self.set_webcam(check_cameras, int(video_info))
        elif self._camera_mode is VideoMode.IP_CAM:
            self.set_ipcam(video_info)
        elif self._camera_mode is VideoMode.DISK_VIDEO:
            self.set_disk_video(video_info)
            self._disk_mode = True

        if not self._cap.isOpened() or self._cap is None:
            self.set_youtube_stream()
            self._youtube_mode = True

    def start_camera_publisher(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            ret, frame = self._cap.read()

            if not ret:
                if self._youtube_mode:
                    self._cap.open(self._video_stream.url)
                    continue
                elif self._disk_mode:
                    self._cap.open(self._disk_path)
                    continue

                rospy.logerr('Could not get image')
                raise IOError('[CameraNode] Could not get image')

            msg = Image()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self._dev_id
            #if self._youtube_mode:
                #msg.header.frame_id = 'dev1'
            msg.encoding = "bgr8"
            msg.data = np.array(frame, dtype=np.uint8).tobytes()
            msg.height, msg.width = frame.shape[:-1]
            msg.step = frame.shape[-1]*frame.shape[0]
            self._pub.publish(msg)
            rate.sleep()

    def set_youtube_stream(self, url = "https://youtu.be/bqpCkbAr8dY"):
        video = pafy.new(url)
        self._video_stream = video.getbest(preftype="mp4")
        self._cap = cv2.VideoCapture()
        self._cap.open(self._video_stream.url)

    def set_webcam(self, check_cameras = False, camera_index = 0):
        if check_cameras:
            indices = returnCameraIndices()
            if len(indices) == 0:
                rospy.logerr('No Cameras Found')
                raise IOError('[CameraNode] No Cameras Found')
            self._cap = cv2.VideoCapture(indices[0])
            if self._verbose:
                print(f"[CameraNode] start cam on index [indices[0]]")
        else:
            self._cap = cv2.VideoCapture(camera_index)

    def set_ipcam(self, camera_ip = None):
        self._cap = cv2.VideoCapture(f"rtsp://admin:Vergessen1@{camera_ip}")
        print(f"START IP: {camera_ip}")

    def set_disk_video(self, path = "/home/trainerai/trainerai-core/data/video.avi"):
        print("TEEST", path)
        self._disk_path = path
        self._cap = cv2.VideoCapture(path)
        print("ALLES GUT")

if __name__ == '__main__':
    print(sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    parser.add_argument("-y", "--youtube-id", type=str, help="start on youtube ID")
    parser.add_argument("-c", "--check-cameras", help="Check all camera input id's from 0 to 10. Takes the first working match.", action="store_true")
    parser.add_argument("-i", "--camera-index", type=int, help="Open camera on opencv camera index")
    parser.add_argument("-p", "--ip", type=str, help="Start IP cam on ip")
    parser.add_argument("--disk", type=str, help="Start video from disk path. Relative to root")
    parser.add_argument("-d", "--dev-id", default=0, type=str, help="Ros msgs header transform dev{dev-id}")
    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]

    if last_arg[:2] == "__":
        #print("1")
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        # arg_count > 1:
        #     print(sys.argv[1])
        #     if str(sys.argv[1]).contains(" "):
        args = parser.parse_args()

    #Todo: In Ros Logger
    if args.verbose:
        rospy.loginfo("Verbosity turned on")
        if args.youtube_id is None:
            rospy.loginfo("Youtube turned on")
        if args.camera_index is None:
            rospy.loginfo(f"Try to open {args.camera_index}")
        if args.check_cameras:
            rospy.loginfo("Try to find camera index")
        if args.ip is None:
            rospy.loginfo(f"Try to find ip camera on {args.ip}")
        rospy.loginfo(f"Send on dev{args.dev_id}")

    mode = VideoMode.INVALID
    info = ""
    if args.youtube_id is not None:
        mode = VideoMode.YOUTUBE
        info = args.youtube_id
    elif args.camera_index is not None or args.check_cameras:
        mode = VideoMode.WEB_CAM
        info = args.camera_index
    elif args.ip is not None:
        mode = VideoMode.IP_CAM
        info = args.ip
    elif args.disk is not None:
        mode = VideoMode.DISK_VIDEO
        info = args.disk

    try:
        node = CameraNode(args.verbose, args.dev_id, args.check_cameras, mode, info)
        node.start_camera_publisher()
    except rospy.ROSInterruptException:
        pass

