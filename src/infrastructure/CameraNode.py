import rospy
import cv2
from sensor_msgs.msg import Image
import numpy as np

def camera():
    pub = rospy.Publisher('image', Image, queue_size=2)
    rospy.init_node('camera', anonymous=True)
    cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print('cant read camera', ret)
            continue
        
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'dev0'
        msg.encoding = "bgr8"
        msg.data = np.array(frame, dtype=np.uint8).tostring()
        msg.height, msg.width = frame.shape[:-1]
        msg.step = frame.shape[-1]*frame.shape[0]
        pub.publish(msg)

if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass

