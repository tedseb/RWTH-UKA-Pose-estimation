#!/usr/bin/python3 
import rospy
import cv2
from sensor_msgs.msg import Image
import numpy as np
import pafy
import argparse
import pylint
import sys
# from utils.imutils import crop_bboxInfo, process_image_bbox, process_image_keypoints, bbox_from_keypoints ToDo: Do cropping here. 

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
    def __init__(self, verbose = False, force_youtube = False, check_cameras = False, camera_index = 0):
        self._url = "https://youtu.be/bqpCkbAr8dY"#"https://youtu.be/Z6SKzx7C84M"  https://youtu.be/bqpCkbAr8dY
        self._video_stream = None
        self._cap = None
        self._camera_index = camera_index
        self._verbose = verbose
        self._force_youtube = force_youtube
        self._check_cameras = check_cameras
        self._pub = rospy.Publisher('image', Image, queue_size=2)
        self._rospy.init_node('camera', anonymous=True)

        if force_youtube:
            video = pafy.new(self.url)
            self.video_stream = video.getbest(preftype="mp4")
            self._cap = cv2.VideoCapture()
            self._cap.open(self._video_stream.url)
        else:
            if check_cameras: 
                indices = returnCameraIndices()
                if len(indices) == 0:
                    raise IOError('No Cameras Found')
                self._cap = cv2.VideoCapture(indices[0])  
                if verbose: 
                    print(f"[CameraNode] start cam on index [indices[0]]")
            else:
                self._cap = cv2.VideoCapture(self._camera_index)

    def start_camera_publisher(self):
        rate = self._rospy.Rate(30)

        while not rospy.is_shutdown():
            ret, frame = self._cap.read()
            # if not ret:
            #     try:
            #         self._cap.open(self._video_stream.url)
            #     except:
            #         rospy.loginfo('cant read camera', ret)
            #     continue

            msg = Image()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'dev0'
            msg.encoding = "bgr8"
            msg.data = np.array(frame, dtype=np.uint8).tobytes()
            msg.height, msg.width = frame.shape[:-1]
            msg.step = frame.shape[-1]*frame.shape[0]
            self._pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    parser.add_argument("-y", "--youtube", help="force youtube video", action="store_true")
    parser.add_argument("-c", "--check-cameras", help="check all cam id's from 0 to 10", action="store_true")
    parser.add_argument("-i", "--camera-index", default=0, type=int, help="opencv camera index")
    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]

     
    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()
    
    #Todo: In Ros Logger
    if args.verbose:
        print("[CameraNode] verbosity turned on")

        if args.youtube:
            print("[CameraNode] youtube turned on")

        if args.check_cameras:
            print("[CameraNode] cameras turned on")
    
    try:
        node = CameraNode(args.verbose, args.youtube, args.check_cameras, args.camera_index)
        node.start_camera_publisher()
    except rospy.ROSInterruptException:
        pass

