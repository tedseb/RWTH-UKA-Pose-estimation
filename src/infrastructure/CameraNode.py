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

url = "https://youtu.be/bqpCkbAr8dY"             #"https://youtu.be/Z6SKzx7C84M"  https://youtu.be/bqpCkbAr8dY
video = pafy.new(url)
best = video.getbest(preftype="mp4")
def camera():
    pub = rospy.Publisher('image', Image, queue_size=2)
    rospy.init_node('camera', anonymous=True)
    rate = rospy.Rate(30)
    cap = cv2.VideoCapture(14123)   
    if cap is None or not cap.isOpened():
        rospy.loginfo('INFO: No webCam source. Youtube video will be used as source!')
        cap = cv2.VideoCapture()
        cap.open(best.url)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        #rospy.loginfo("read camera")
        if not ret:
            try:
                cap.open(best.url)
            except:
                rospy.loginfo('cant read camera', ret)
            continue

        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'dev0'
        msg.encoding = "bgr8"
        msg.data = np.array(frame, dtype=np.uint8).tobytes()
        msg.height, msg.width = frame.shape[:-1]
        msg.step = frame.shape[-1]*frame.shape[0]
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    parser.add_argument("-y", "--youtube", help="force youtube video", action="store_true")
    parser.add_argument("-c", "--check-cameras", help="check all cam id's from 0 to 10", action="store_true")
    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]

    print("#####################################")
    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()
    
    if args.verbose:
        print("verbosity turned on")

    if args.youtube:
        print("youtube turned on")

    if args.check_cameras:
        print("cameras turned on")

    print("#####################################")
    
    try:
        camera()
    except rospy.ROSInterruptException:
        pass

