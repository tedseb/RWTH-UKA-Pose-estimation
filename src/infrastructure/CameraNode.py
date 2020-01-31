import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
import sys
import argparse

class CameraNode(Node):
    
    def __init__(self, dev_id=0):
        super().__init__('camera')
        self.pub = self.create_publisher(Image, 'image0', 10)
        timer_period = 0.01
        self.tmr = self.create_timer(timer_period, self.publish_image)
        self.cap = cv2.VideoCapture(dev_id)
        
    def publish_image(self):
        
        ret, frame = self.cap.read()
        if not ret:
            return
            
        msg = Image()
        msg.encoding = "png"
        msg.data = np.array(frame).tostring()
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
