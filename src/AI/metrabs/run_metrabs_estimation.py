#!/usr/bin/env python3
import numpy as np
import cv2
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ["CUDA_VISIBLE_DEVICES"] = "0" # TODO: @sm Check how many CUDA devices there actually are and manage them

import tensorflow as tf
import time
import rospy as rp
from sensor_msgs.msg import Image
from backend.msg import ImageData
from backend.msg import Person, Persons, Bodypart, Pixel,Bboxes
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import logy
import time

import logging
logging.basicConfig(level='ERROR')

from tensorflow.python.keras.backend import set_session

config = tf.compat.v1.ConfigProto()
config.gpu_options.allow_growth = True # dynamically grow the memory used on the GPU
config.log_device_placement = True # to log device placement (on which device the operation ran)
sess = tf.compat.v1.Session(config=config)
set_session(sess)

import matplotlib.pyplot as plt
plt.switch_backend('TkAgg')
from mpl_toolkits.mplot3d import Axes3D

from inspect import getmembers, isfunction

CONFIG = {
    'intrinsics': [[1962, 0, 540], [0, 1969, 960], [0, 0, 1]], # [[3324, 0, 1311], [0, 1803, 707], [0, 0, 1]]
    'model_path': '/home/trainerai/trainerai-core/src/AI/metrabs/models/metrabs_multiperson_smpl' # /home/trainerai/trainerai-core/src/AI/metrabs/models/metrabs_multiperson_smpl_combined
}

class PoseEstimator():
    def __init__(self):
        self.model = tf.saved_model.load(CONFIG["model_path"])

        self.intrinsics = tf.constant(CONFIG.get("intrinsics"), dtype=tf.float32)
        image = tf.image.decode_jpeg(tf.io.read_file('/home/trainerai/trainerai-core/src/AI/metrabs/test_image_3dpw.jpg'))
        person_boxes = tf.constant([ [1000, 350, 500, 650]], tf.float32)

        self.model.predict_single_image(image, self.intrinsics, person_boxes)


        # Use your detector of choice to obtain bounding boxes.
        # See the README for how to combine the YOLOv4 detector with our MeTRAbs code.

        # define a publisher to publish the 3D skeleton of multiple people
        self.publisher = rp.Publisher('personsJS', Persons, queue_size=2)
        self.publisher_crop = rp.Publisher('cropped_images', Image, queue_size=2)

        # Define a CV bridge that handles images for us
        self.opencv_bridge = CvBridge()

        # define a subscriber to retrive tracked bodies
        rp.Subscriber('bboxes', Bboxes, self.callback_regress)
        #rp.Subscriber('bboxes1', Bboxes, self.callback_regress)

        rp.Subscriber('image', ImageData, self.callback_setImage)
        #srp.Subscriber('image1', Image, self.callback_setImage)


    def callback_setImage(self, msg : ImageData):
        #logy.info_throttle("GET IMAGE", 2000)
        #logy.info_throttle("GET IMAGE", 2000)
        if msg.is_debug:
            logy.debug(f"Received image. Debug frame {msg.debug_id}", tag="debug_frame")
        self.last_image_message = msg.image

    def callback_regress(self, body_bbox_list_station : Bboxes):
        if body_bbox_list_station.is_debug:
            logy.debug(f"Received bboxes. Debug frame {body_bbox_list_station.debug_id}", tag="debug_frame")
        body_bbox_list_station_reshaped = np.array(body_bbox_list_station.data).reshape(-1,4)
        tmpTime = time.time()
        # TODO: Differ between someone that is focused on the station and someone that is going through the camera and let to occlusion. Currently take the skeleton that is the biggest

        if not hasattr(self, "last_image_message"):
            rp.logerr_throttle(5, "Pose Estimator has no image input. If you see this message multiple times there is something wrong.")
            return

        height = self.last_image_message.height
        width = self.last_image_message.width
        image = np.frombuffer(self.last_image_message.data, dtype=np.uint8)
        image = image.reshape([height, width, 3])    #(480, 640, 3) --> (y,x,3) = (h,w,3)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Body Pose Regression
        pred_output_list = self.model.predict_single_image(image, self.intrinsics, body_bbox_list_station_reshaped)
        poses2d = ((pred_output_list / pred_output_list[..., 2:]) @ tf.linalg.matrix_transpose(self.intrinsics))[..., :2]
        stationID = body_bbox_list_station.stationID
        sensorID = body_bbox_list_station.sensorID
        boxes = body_bbox_list_station_reshaped

        if len(pred_output_list.numpy()) == 0:
            rp.logerr_throttle(5, "Station is active but Pose Estimator could not detect people.")
            return

        inc = 0
        msg = Persons()
        msg.header = self.last_image_message.header
        msg.persons = list()
        cropped_images = []
        for idx,detection in enumerate(pred_output_list.numpy()):
            joints=detection
            bb=boxes[idx]
            image = image[int(bb[1]):int(bb[1]+bb[3]),int(bb[0]):int(bb[0]+bb[2])]
            cropped_images.append(image)

            lenPoints=len(joints)       # TODO: use fixed number of joints to save calculation time
            person_msg = Person()
            if len(stationID) > 0:
                person_msg.stationID = int(stationID[inc])
                person_msg.sensorID = sensorID[inc]
            person_msg.bodyParts = [None]*lenPoints
            for idx in range(lenPoints):
                person_msg.bodyParts[idx] = Bodypart()
            for idx in range(lenPoints):
                person_msg.bodyParts[idx].score = 0.8
                person_msg.bodyParts[idx].pixel.x = joints[idx, 0]
                person_msg.bodyParts[idx].pixel.y = joints[idx, 1]
                person_msg.bodyParts[idx].point.x =joints[idx, 0] / 400-6 # TODO: @sm Please describe what these number stand for
                person_msg.bodyParts[idx].point.y =joints[idx, 2] / 400-25
                person_msg.bodyParts[idx].point.z = -joints[idx, 1] / 400
            inc=inc+1
            msg.persons.append(person_msg)
        self.publisher.publish(msg)
        # Concatenate images and convert them to ROS image format to display them later in rviz
        image_message = self.opencv_bridge.cv2_to_imgmsg(cv2.vconcat(cropped_images), encoding="passthrough")

        self.publisher_crop.publish(image_message)


if __name__ == '__main__':
    logy.basic_config(debug_level=logy.DEBUG, module_name="PE")

    rp.init_node('metrabs', anonymous=True)
    run_spin_obj = PoseEstimator()
    logy.info("Pose Estimator is listening")
    rp.spin()
