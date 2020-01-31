import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
import sys
import argparse
import time

class CameraNode(Node):
    
    def __init__(self):
        super().__init__('camera')
        self.pub = self.create_publisher(Image, 'image0', 10)
        timer_period = 0.01
        self.tmr = self.create_timer(timer_period, self.publish_image)
        self.cap = cv2.VideoCapture(0)
        
    def publish_image(self):
        
        ret, frame = self.cap.read()
        if not ret:
            return
            
        msg = Image()
        msg.header.frame_id = 'dev0'
        t = time.time()
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int(t-int(t))
        msg.encoding = "png"
        msg.data = np.array(frame, dtype=np.uint8).tostring()
        msg.height, msg.width = frame.shape[:-1]
        msg.step = frame.shape[-1]*frame.shape[1]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode(0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    #parser = argparse.ArgumentParser(description='Publishes camera images to ros2 topics.')
    #parser.add_argument('device', dest='device', action='store_const', default=0, help='specifies the device, which is read. Default: 0')
    #args = parser.parse_args()
    main()
