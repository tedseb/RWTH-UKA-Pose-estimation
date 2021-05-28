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

videos_mit_ted = {
"deadlift_good": "https://www.youtube.com/watch?v=tZvIbg5SiB0",
"deadlift_bad": "https://www.youtube.com/watch?v=HAvKEgNne_s",
"jumps": "https://www.youtube.com/watch?v=6Jmuqqzdngg",
"langhantel_curls_bad": "https://www.youtube.com/watch?v=BzpF-yAIstU",
"langhantel_curls_good": "https://www.youtube.com/watch?v=gTux94rB92w",
"military_press_bad": "https://www.youtube.com/watch?v=EsZ_GA8Pk7k",
"military_press_good": "https://www.youtube.com/watch?v=QVQ5WaEKyFU",
"pullups_bad": "https://www.youtube.com/watch?v=LujrsivJmbQ",
"pullups_good": "https://www.youtube.com/watch?v=YiVXPZQhHSA",
"orhan_ted_buero": "https://youtu.be/bqpCkbAr8dY",
}

class CameraNode():
    def __init__(self, verbose = False, force_youtube = False, check_cameras = False, video_string="deadlift_good", camera_index = 0):
        # Try to get the Video from the video list, if the video can not be found, the video string is probably a video link itself
        self._url = videos_mit_ted.get(video_string, video_string)

        self._cap = None
        self._camera_index = camera_index
        self._verbose = verbose
        self._force_youtube = force_youtube
        self._check_cameras = check_cameras
        rospy.init_node('camera', anonymous=True)
        self._pub = rospy.Publisher('image', Image, queue_size=2)
        
        if force_youtube:
            self.set_youtube_stream()
        else:
            self.set_camera()

        if not self._cap.isOpened() or self._cap is None: 
            self.set_youtube_stream()
            self._force_youtube = True

    def start_camera_publisher(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            ret, frame = self._cap.read()
            
            if not ret:
                if self._force_youtube: 
                    self._cap.open(self._video_stream.url)
                    continue
                rospy.logerr('Could not get image')
                raise IOError('[CameraNode] Could not get image')

            msg = Image()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'dev0'
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

    def set_camera(self):
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

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    parser.add_argument("-y", "--youtube", help="force youtube video", action="store_true")
    parser.add_argument("-c", "--check-cameras", help="check all cam id's from 0 to 10", action="store_true")
    parser.add_argument("-i", "--camera-index", default=0, type=int, help="opencv camera index")
    parser.add_argument("-s", "--video-string", default=None, type=str, help="string of the video we want to play, see CameraNode.py")
    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]

     
    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()
    
    #Todo: In Ros Logger
    if args.verbose:
        print("[CameraNode] Verbosity turned on")
        rospy.loginfo("Verbosity turned on")
        if args.youtube:
            print("[CameraNode] Youtube turned on")
            rospy.loginfo("Youtube turned on")
        else:
            print(f"[CameraNode] Try to open {args.camera_index}")
            rospy.loginfo(f"Try to open {args.camera_index}")

        if args.check_cameras:
            print("[CameraNode] Try to find camera index")
            rospy.loginfo("Try to find camera index")

    try:
        node = CameraNode(args.verbose, args.youtube, args.check_cameras, args.video_string, args.camera_index)
        node.start_camera_publisher()
    except rospy.ROSInterruptException:
        pass

