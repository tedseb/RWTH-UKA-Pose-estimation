import argparse
import sys
import time
import rospy
import numpy as np
from backend.srv import WeightDetection as RosWeightDetection
from backend.srv import WeightDetectionResponse, WeightDetectionRequest
from sensor_msgs.msg import Image

VERBOSE_MODE = True
def LOG_DEBUG(msg, debug = VERBOSE_MODE):
    if debug:
        rospy.loginfo(f"[WD] {msg}", logger_name="StationManager")

def LOG_INFO(msg):
    rospy.loginfo(f"[WD] {msg}", logger_name="StationManager")

class WeightDetection:
    def __init__(self, debug=False, verbose=False):
        self._verbose = verbose
        self._debug = debug
        self._weigth_detection = rospy.Service('sm/weight_detection', RosWeightDetection, self.handle_weight_detection_request)
        print("init")

    def start(self):
        LOG_DEBUG("Started StationManager", self._verbose)
        rospy.spin()

    def handle_weight_detection_request(self, req : WeightDetectionRequest):
        image_channel = "Image"
        time_limit = 2
        sub = rospy.Subscriber(image_channel, Image, self.process_image)
        time.sleep(time_limit)
        print("test")

    def process_image(self, img_msg):
        shape = img_msg.height, img_msg.width, 3                            #(480, 640, 3) --> (y,x,3)
        img = np.frombuffer(img_msg.data, dtype=np.uint8)
        img_original_bgr = img.reshape(shape)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="Verbose mode", action="store_true")
    ARG_COUNT = len(sys.argv)
    last_arg = sys.argv[ARG_COUNT - 1]
    if last_arg[:2] == "__":
        valid_args = sys.argv[1:ARG_COUNT - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()

    weigh_detection = WeightDetection(debug_mode=args.debug, verbose=args.verbose)
    weigh_detection.start()
