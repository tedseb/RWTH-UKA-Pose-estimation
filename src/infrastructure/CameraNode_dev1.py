#!/usr/bin/python3 
import rospy
import cv2
from sensor_msgs.msg import Image
import numpy as np
import pafy
# from utils.imutils import crop_bboxInfo, process_image_bbox, process_image_keypoints, bbox_from_keypoints ToDo: Do cropping here. 

url = "https://youtu.be/BHeko6p-JoY"             #"https://www.youtube.com/watch?v=C_VtOYc6j5c"  https://www.youtube.com/watch?v=QifjltKUMCk
video = pafy.new(url)
best = video.getbest(preftype="mp4")
def camera():
    pub = rospy.Publisher('image1', Image, queue_size=2)
    rospy.init_node('camera', anonymous=True)
    rate = rospy.Rate(30)
    cap = cv2.VideoCapture(6)   
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
        msg.header.frame_id = 'dev1'
        msg.encoding = "bgr8"
        msg.data = np.array(frame, dtype=np.uint8).tostring()
        msg.height, msg.width = frame.shape[:-1]
        msg.step = frame.shape[-1]*frame.shape[0]
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
        

    try:
        camera()
    except rospy.ROSInterruptException:
        pass
