#!/usr/bin/python3 
import rospy
import cv2
from sensor_msgs.msg import Image
import numpy as np
import pafy
import argparse
import pylint
import sys

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
    def __init__(self, verbose = False, dev_id = 0, youtube_id = "", check_cameras = False, camera_index = -1, camera_ip = "", ):
        self._cap = None
        self._camera_index = camera_index
        self._camera_ip = camera_ip
        self._check_cameras = check_cameras
        self._verbose = verbose
        self._camera_mode = -1
        self._dev_id = "dev" + str(dev_id)
        
        rospy.init_node('camera', anonymous=True)
        self._pub = rospy.Publisher('image', Image, queue_size=1)

        if youtube_id != "":
            self._url = f"https://www.youtube.com/watch?v={youtube_id}"
            self._youtube_mode = True
            self.set_youtube_stream()
            self._camera_mode = 0
        elif check_cameras or camera_index >= 0: 
            self._youtube_mode = False
            self.set_webcam()
            self._camera_mode = 1
        elif camera_ip != "":
            self._youtube_mode = False
            self.set_ipcam()
            self._camera_mode = 2
            #self.set_video()

        if not self._cap.isOpened() or self._cap is None: 
            self.set_youtube_stream()
            self._camera_mode = 0
            self._youtube_mode = True

    def start_camera_publisher(self):
        rate = rospy.Rate(25)  #TODO: Aufnahme ist in 25FPS
        while not rospy.is_shutdown():
            ret, frame = self._cap.read()
            frame = cv2.resize(frame, (1280,720))
            if not ret:
                if self._youtube_mode: 
                    self._cap.open(self._video_stream.url)
                    continue
                rospy.logerr('Could not get image')
                raise IOError('[CameraNode] Could not get image')
            msg = Image()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self._dev_id
            msg.encoding = "bgr8"
            msg.data = np.array(frame, dtype=np.uint8).tobytes()
            msg.height, msg.width = frame.shape[:-1]
            msg.step = frame.shape[-1]*frame.shape[0]
            self._pub.publish(msg)
            rate.sleep()
    
    def set_youtube_stream(self):
        video = pafy.new(self._url)
        self._video_stream = video.getbest(preftype="mp4")
        self._cap = cv2.VideoCapture()
        self._cap.open(self._video_stream.url)

    def set_webcam(self):
        if self._check_cameras: 
            indices = returnCameraIndices()
            if len(indices) == 0:
                rospy.logerr('No Cameras Found')
                raise IOError('[CameraNode] No Cameras Found')
            self._cap = cv2.VideoCapture(indices[0])  
            if self._verbose: 
                print(f"[CameraNode] start cam on index [indices[0]]")
        else:
            self._cap = cv2.VideoCapture(self._camera_index)

    def set_ipcam(self):
        self._cap = cv2.VideoCapture(f"rtsp://admin:Vergessen1@{self._camera_ip}")

    def set_video(self):
        self._cap = cv2.VideoCapture("/home/trainerai/trainerai-core/deaflifts.avi")

if __name__ == '__main__':
    print(sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    parser.add_argument("-y", "--youtube-id", default="", type=str, help="start on youtube ID")
    parser.add_argument("-c", "--check-cameras", help="Check all camera input id's from 0 to 10. Takes the first working match.", action="store_true")
    parser.add_argument("-i", "--camera-index", default=-1, type=int, help="Open camera on opencv camera index")
    parser.add_argument("-p", "--ip", default="", type=str, help="Start IP cam on ip")
    parser.add_argument("-d", "--dev-id", default=0, type=str, help="Ros msgs header transform dev{dev-id}")
    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]
     
    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()
    
    if args.verbose:
        rospy.loginfo("Verbosity turned on")
        if args.youtube_id != "":
            rospy.loginfo("Youtube turned on")
        if args.camera_index >= 0:
            rospy.loginfo(f"Try to open {args.camera_index}")
        if args.check_cameras:
            rospy.loginfo("Try to find camera index")
        if args.ip != "":
            rospy.loginfo(f"Try to find ip camera on {args.ip}")
        rospy.loginfo(f"Send on dev{args.dev_id}")
    
    try:
        node = CameraNode(args.verbose, args.dev_id, args.youtube_id, args.check_cameras, args.camera_index, args.ip)
        node.start_camera_publisher()
    except rospy.ROSInterruptException:
        pass

