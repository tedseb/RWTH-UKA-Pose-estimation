import argparse
import sys
import time
import rospy
import numpy as np
import cv2
from backend.srv import WeightDetection as RosWeightDetection
from backend.srv import WeightDetectionResponse, WeightDetectionRequest
from rospy.exceptions import ROSException
from sensor_msgs.msg import Image
from src.yolo_model import YoloWeightModel
from src.utils import crop_weights_into_image
from src.dumbbell import Dumbbell
#pylint: disable=unused-wildcard-import
from src.config import *

VERBOSE_MODE = True
def LOG_DEBUG(msg, debug = VERBOSE_MODE):
    if debug:
        rospy.loginfo(f"[WD] {msg}", logger_name="StationManager")

def LOG_INFO(msg):
    rospy.loginfo(f"[WD] {msg}", logger_name="StationManager")

def LOG_ERROR(msg):
    rospy.logerr(f"[WD] {msg}", logger_name="StationManager")

def ros_image_to_cv_image(img):
    shape = img.height, img.width, 3
    img = np.frombuffer(img.data, dtype=np.uint8)
    img = img.reshape(shape)
    return img

class WeightDetection:
    def __init__(self, debug=False, verbose=False):
        self._verbose = verbose
        self._debug = debug
        self._weigth_detection = rospy.Service('sm/weight_detection', RosWeightDetection, self.handle_weight_detection_request)
        self._model = YoloWeightModel()
        print("init")

    def start(self):
        LOG_DEBUG("Started StationManager", self._verbose)
        rospy.spin()

    def handle_weight_detection_request(self, req : WeightDetectionRequest):
        image_channel = "Image"
        time_limit = 2

        #Todo: Mehrere Frames warten
        try:
            frame = rospy.wait_for_message(image_channel, Image, timeout=5)
        except ROSException:
            LOG_ERROR("No Image on the channel '{image_channel}'")

        frame = ros_image_to_cv_image(frame)
        frame = cv2.resize(frame, (1280, 720))
        output = self._model.inference(frame)
        pred = output.pred

        if self._model.is_valid_prediction(pred):
            left_box = self._model.get_left_box(pred)
            right_box = self._model.get_right_box(pred)
            dumbbell = Dumbbell(left_box, right_box, frame)
        else:
            LOG_ERROR("No Valid Prediction")

        dumbbell = Dumbbell(left_box, right_box, frame)
        sub = rospy.Subscriber(image_channel, Image, self.process_image, callback_args=(dumbbell))
        time.sleep(time_limit)
        sub.unregister()

        barcode_left, barcode_right = dumbbell.barcodes()

        weight_left = 0
        for code in barcode_left:
            weight_left += COLOR_RANGES[code][1]

        weight_right = 0
        for code in barcode_right:
            weight_right += COLOR_RANGES[code][1]

        return WeightDetectionResponse(weight_left + weight_right)

    def process_image(self, img_msg, dumbbell : Dumbbell):
        img = ros_image_to_cv_image(img_msg)
        img = cv2.resize(img, (1280, 720))
        dumbbell.add_masks(img)

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
