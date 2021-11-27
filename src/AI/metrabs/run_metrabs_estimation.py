#!/usr/bin/env python3
import numpy as np
import cv2
import os
from multiprocessing import Lock

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ["CUDA_VISIBLE_DEVICES"] = "0" # TODO: @sm Check how many CUDA devices there actually are and manage them

import tensorflow as tf
import time
import rospy
from collections import deque
from sensor_msgs.msg import Image
from backend.msg import ImageData, ChannelInfo
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

class Queue:
    def __init__(self, queue_len = 10):
        self._elements = deque([])
        self._mutex = Lock()
        self._queue_len = queue_len

    def put(self, item):
        with self._mutex:
            self._elements.append(item)
            if len(self._elements) > self._queue_len:
                self._elements.popleft()

    def get(self):
        with self._mutex:
            item = self._elements.popleft()
        return item

    def empty(self):
        if self._elements:
            return False
        return True

    def __getitem__(self, key):
        with self._mutex:
            item = self._elements[key]
        return item

class PoseEstimator():
    def __init__(self):
        rospy.Subscriber('bboxes', Bboxes, self.callback_regress, queue_size=1)
        rospy.Subscriber('/channel_info', ChannelInfo, self.handle_new_channel)
        self.model = tf.saved_model.load(CONFIG["model_path"])

        self.intrinsics = tf.constant(CONFIG.get("intrinsics"), dtype=tf.float32)



        # Use your detector of choice to obtain bounding boxes.
        # See the README for how to combine the YOLOv4 detector with our MeTRAbs code.

        # define a publisher to publish the 3D skeleton of multiple people
        self.publisher = rospy.Publisher('personsJS', Persons, queue_size=2)
        self.publisher_crop = {}

        self.subscriber_image = {}


        # Define a CV bridge that handles images for us
        self.opencv_bridge = CvBridge()
        self._send_cropped = True
        # define a subscriber to retrive tracked bodies


        #rospy.Subscriber('bboxes1', Bboxes, self.callback_regress)

        #rospy.Subscriber('image', ImageData, self.callback_setImage)
        self._image_queue = Queue(3)
        self._debug_time_queue = Queue(20)
        #srospy.Subscriber('image1', Image, self.callback_setImage)

        image = tf.image.decode_jpeg(tf.io.read_file('/home/trainerai/trainerai-core/src/AI/metrabs/test_image_3dpw.jpg'))
        person_boxes = tf.constant([ [1000, 350, 500, 650]], tf.float32)
        self.model.predict_single_image(image, self.intrinsics, person_boxes)


    @logy.catch_ros
    def handle_new_channel(self, channel_info: ChannelInfo):
        if channel_info.is_active:
            logy.debug(f"New Channel: {channel_info.channel_name}")
            if channel_info.cam_id not in self.subscriber_image:
                sub = rospy.Subscriber(channel_info.channel_name, ImageData, self.callback_setImage)
                self.subscriber_image[channel_info.cam_id] = sub
            if channel_info.cam_id not in self.publisher_crop and self._send_cropped:
                pub = rospy.Publisher(f'{channel_info.channel_name}_cropped', Image , queue_size=2)
                self.publisher_crop[channel_info.cam_id] = pub
        else:
            if channel_info.cam_id in self.subscriber_image:
                self.subscriber_image[channel_info.cam_id].unregister()
                del self.subscriber_image[channel_info.cam_id]
            if channel_info.cam_id in self.publisher_crop and self._send_cropped:
                self.publisher_crop[channel_info.cam_id].unregister()
                del self.publisher_crop[channel_info.cam_id]

    def callback_setImage(self, msg: ImageData):
        #logy.debug_throttle("Received Image", 2000)
        if msg.is_debug:
            logy.debug(f"Received image. Debug frame {msg.debug_id}", tag="debug_frame")
            time_ms = time.time() * 1000
            self._debug_time_queue.put((time_ms, msg.debug_id))
        logy.log_fps("metraps_image_fps")
        self._image_queue.put(msg)

    def get_next_image_in_queue(self, box_frame_number: int):
        if self._image_queue.empty():
            return None

        bbox_num = box_frame_number
        next_img_num = self._image_queue[0].frame_num

        while next_img_num <= bbox_num:
            img_data = self._image_queue.get()
            if self._image_queue.empty():
                break
            next_img_num = self._image_queue[0].frame_num

        if next_img_num != bbox_num:
            return None
        return img_data

    def get_next_debug_frame(self, box_debug_id: int):
        if self._debug_time_queue.empty():
            return None

        bbox_num = box_debug_id
        next_img_num = self._debug_time_queue[0][1]
        while next_img_num <= bbox_num:
            debug_data = self._debug_time_queue.get()
            if self._debug_time_queue.empty():
                break
            next_img_num = self._debug_time_queue[0][1]

        if next_img_num != bbox_num:
            return None
        return debug_data

    @logy.catch_ros
    def callback_regress(self, body_bbox_list_station: Bboxes):
        #logy.debug_throttle("Received Bbox", 2000)
        img_data = self.get_next_image_in_queue(body_bbox_list_station.frame_num)
        if img_data is None:
            #logy.warn_throttle("The next image is missing")
            return
        last_image = img_data.image

        if body_bbox_list_station.is_debug:
            debug_info = self.get_next_debug_frame(body_bbox_list_station.debug_id)
            if debug_info is None:
                logy.warn("The next Debug image is missing.")
                logy.warn("If you can see this message, there is something fundamental wrong in the image queue.")
            else:
                logy.debug(f"Received bboxes. Debug frame {body_bbox_list_station.debug_id}", tag="debug_frame")
                time_ms = time.time() * 1000
                time_ms = time_ms - debug_info[0]
                logy.log_avg("debug_frame_delay", time_ms)
                logy.log_mean("debug_frame_delay", time_ms)

        body_bbox_list_station_reshaped = np.array(body_bbox_list_station.data).reshape(-1,4)
        logy.log_fps("metraps_bbox_fps")

        # TODO: Differ between someone that is focused on the station and someone that is going through the camera and let to occlusion. Currently take the skeleton that is the biggest
        height = last_image.height
        width = last_image.width
        image = np.frombuffer(last_image.data, dtype=np.uint8)
        image = image.reshape([height, width, 3])    #(480, 640, 3) --> (y,x,3) = (h,w,3)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Body Pose Regression
        pred_output_list = self.model.predict_single_image(image, self.intrinsics, body_bbox_list_station_reshaped)
        poses2d = ((pred_output_list / pred_output_list[..., 2:]) @ tf.linalg.matrix_transpose(self.intrinsics))[..., :2]
        stationID = body_bbox_list_station.stationID
        sensorID = body_bbox_list_station.sensorID
        boxes = body_bbox_list_station_reshaped

        if len(pred_output_list.numpy()) == 0:
            rospy.logerr_throttle(5, "Station is active but Pose Estimator could not detect people.")
            return

        inc = 0
        msg = Persons()
        msg.header = last_image.header
        msg.persons = list()
        cropped_images = []
        for idx,detection in enumerate(pred_output_list.numpy()):
            joints=detection
            bb=boxes[idx]
            cropped_image = image[int(bb[1]):int(bb[1]+bb[3]),int(bb[0]):int(bb[0]+bb[2])]
            cropped_images.append(cropped_image)

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

        if len(cropped_images) > 1:
            max_h = 0
            for img in cropped_images:
                max_h = max(max_h, img.shape[0])


            for i, img in enumerate(cropped_images):
                height_difference = max_h - img.shape[0]
                cropped_images[i] = cv2.copyMakeBorder(img, height_difference, 0, 0, 0, cv2.BORDER_CONSTANT | cv2.BORDER_ISOLATED, (0, 0, 0))

        # Concatenate images and convert them to ROS image format to display them later in rviz
        cam_id = int(body_bbox_list_station.header.frame_id[3:])
        img = cv2.hconcat(cropped_images)
        image_message = self.opencv_bridge.cv2_to_imgmsg(img, encoding="passthrough")
        self.publisher_crop[cam_id].publish(image_message)


if __name__ == '__main__':
    logy.basic_config(debug_level=logy.DEBUG, module_name="PE")

    rospy.init_node('metrabs', anonymous=True)
    run_spin_obj = PoseEstimator()
    logy.info("Pose Estimator is listening")
    rospy.spin()
