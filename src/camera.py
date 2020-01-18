import rospy
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np

def camera():
    pub = rospy.Publisher('images', CompressedImage, queue_size=2)
    rospy.init_node('camera', anonymous=True)
    cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print('cant read camera', ret)
            continue
        
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        pub.publish(msg)

if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass
