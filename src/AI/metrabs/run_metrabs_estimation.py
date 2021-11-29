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
import threading
import logging
from multiprocessing import Lock
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

THREAD_WAIT_TIME_MS = 20 #40 ms are the time beween two images at 25fps
AI_HEIGHT = 720
AI_WIDTH = 1280

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

        self.intrinsics = tf.constant([CONFIG.get("intrinsics")], dtype=tf.float32)
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

        self._image_queues = {}
        self._debug_time_queues = {}
        self._box_queues = {}
        self._thread_lock = Lock()

        image = np.empty([AI_HEIGHT, AI_WIDTH, 3], dtype=np.uint8)
        person_boxes = [np.array([100, 100, 100, 100], np.float32)]
        self.start_ai([], np.stack([image]), [person_boxes])

        self._is_active = True
        self._ai_thread = threading.Thread(target=self.thread_handler)
        self._ai_thread.start()

    def __del__(self):
        self._is_active = False
        self._ai_thread.join(timeout=1)

    @logy.catch_ros
    def handle_new_channel(self, channel_info: ChannelInfo):
        with self._thread_lock:
            if channel_info.is_active:
                logy.debug(f"New Channel: {channel_info.channel_name}")
                if channel_info.cam_id not in self.subscriber_image:
                    sub = rospy.Subscriber(channel_info.channel_name, ImageData, self.callback_setImage)
                    self.subscriber_image[channel_info.cam_id] = sub
                if channel_info.cam_id not in self.publisher_crop and self._send_cropped:
                    pub = rospy.Publisher(f'{channel_info.channel_name}_cropped', Image , queue_size=2)
                    self.publisher_crop[channel_info.cam_id] = pub
                if channel_info.cam_id not in self._image_queues:
                    self._image_queues[channel_info.cam_id] = Queue(5)
                if channel_info.cam_id not in self._debug_time_queues:
                    self._debug_time_queues[channel_info.cam_id] = Queue(20)
                if channel_info.cam_id not in self._box_queues:
                    self._box_queues[channel_info.cam_id] = Queue(2)
            else:
                if channel_info.cam_id in self.subscriber_image:
                    self.subscriber_image[channel_info.cam_id].unregister()
                    del self.subscriber_image[channel_info.cam_id]
                if channel_info.cam_id in self.publisher_crop and self._send_cropped:
                    self.publisher_crop[channel_info.cam_id].unregister()
                    del self.publisher_crop[channel_info.cam_id]
                if channel_info.cam_id in self._image_queues:
                    del self._image_queues[channel_info.cam_id]
                if channel_info.cam_id in self._debug_time_queues:
                    del self._debug_time_queues[channel_info.cam_id]
                if channel_info.cam_id in self._box_queues:
                    del self._box_queues[channel_info.cam_id]

    @logy.catch_ros
    def callback_setImage(self, msg: ImageData):
        camera_id = int(msg.image.header.frame_id[3:])
        #logy.debug_throttle("Received Image", 2000)
        if msg.is_debug:
            logy.debug(f"Received image. Debug frame {msg.debug_id}", tag="debug_frame")
            time_ms = time.time() * 1000
            self._debug_time_queues[camera_id].put((time_ms, msg.debug_id))

        if camera_id in self._image_queues:
            self._image_queues[camera_id].put(msg)
            logy.log_fps("metraps_image_fps", 50, 0)

    def get_next_image_in_queue(self, box_frame_number: int, camera_id: int):
        queue = self._image_queues.get(camera_id)
        if queue is None or queue.empty():
            return None

        bbox_num = box_frame_number
        next_img_num = queue[0].frame_num

        while next_img_num <= bbox_num:
            img_data = queue.get()
            if queue.empty():
                break
            next_img_num = queue[0].frame_num

        if next_img_num != bbox_num:
            return None
        return img_data

    def get_next_debug_frame(self, box_debug_id: int, camera_id: int):
        queue = self._debug_time_queues.get(camera_id)
        if queue is None or queue.empty():
            return None

        next_debug_id = queue[0][1]
        while next_debug_id <= box_debug_id:
            debug_data = queue.get()
            if queue.empty():
                break
            next_debug_id = queue[0][1]

        if next_debug_id != box_debug_id:
            return None
        return debug_data

    @logy.catch_ros
    def callback_regress(self, body_bbox_list_station: Bboxes):
        camera_id = int(body_bbox_list_station.header.frame_id[3:])
        if camera_id in self._box_queues:
                self._box_queues[camera_id].put(body_bbox_list_station)

    def thread_handler(self):
        logy.debug("Metrabs Thread is Active")
        thread_wait_time = THREAD_WAIT_TIME_MS / 1000.0
        while(self._is_active):
            boxes = []
            images = []
            intrinsics = []
            data = []
            with self._thread_lock:
                for camera_id, queue in self._box_queues.items():
                    if queue.empty():
                        continue

                    box = queue.get()
                    box_np = np.array(box.data).reshape(-1, 4)
                    if box_np.size == 0:
                        continue

                    img_data = self.get_next_image_in_queue(box.frame_num, camera_id)
                    if img_data is None:
                        #logy.warn("The next image is missing")
                        continue

                    if box.is_debug:
                        debug_info = self.get_next_debug_frame(box.debug_id, camera_id)
                        if debug_info is None:
                            logy.warn("The next Debug image is missing.")
                            logy.warn("If you can see this message, there is something fundamental wrong in the image queue.")
                        else:
                            logy.debug(f"Received bboxes. Debug frame {box.debug_id}", tag="debug_frame")
                            time_ms = time.time() * 1000
                            time_ms = time_ms - debug_info[0]
                            logy.log_avg("debug_frame_delay", time_ms)
                            logy.log_mean("debug_frame_delay", time_ms)

                    last_image = img_data.image
                    height = last_image.height
                    width = last_image.width
                    image = np.frombuffer(last_image.data, dtype=np.uint8)
                    image = image.reshape([height, width, 3])
                    if width != AI_WIDTH or height != AI_HEIGHT:
                        image = cv2.resize(image, (AI_WIDTH, AI_HEIGHT))
                        w_factor = AI_WIDTH / width
                        h_factor = AI_HEIGHT / height
                        box_np[0, 0] *= w_factor
                        box_np[0, 1] *= h_factor
                        box_np[0, 2] *= w_factor
                        box_np[0, 3] *= h_factor

                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    images.append(image)
                    boxes.append(box_np.astype(np.float32))
                    data.append((camera_id, box.stationID, last_image.header))

            if len(data) == 0:
                time.sleep(thread_wait_time)
                continue

            images = np.stack(images)
            self.start_ai(data, images, boxes)

    def start_ai(self, data, images, boxes):
        ragged_boxes = tf.ragged.constant(boxes, ragged_rank=1)

        with logy.TraceTime("matrabs_multi_image"):
            pred_output_list = self.model.predict_multi_image(images, self.intrinsics, ragged_boxes)

        pred_output_list = pred_output_list.numpy()

        #logy.warn(f"RESULT: {pred_output_list}")

        for i, info in enumerate(data):
            camera_id = info[0]
            station_ids = info[1]
            image_boxes = boxes[i]
            image = images[i]

            inc = 0
            msg = Persons()
            msg.header = info[2]
            msg.persons = list()
            cropped_images = []

            if len(pred_output_list[i]) == 0:
                rospy.logerr_throttle(5, "Station is active but Pose Estimator could not detect people.")
                continue

            for prediction_index, detection in enumerate(pred_output_list[i]):

                joints = detection
                bb = image_boxes[prediction_index]
                cropped_image = image[int(bb[1]):int(bb[1]+bb[3]),int(bb[0]):int(bb[0]+bb[2])]
                cropped_images.append(cropped_image)

                lenPoints=len(joints)       # TODO: use fixed number of joints to save calculation time
                person_msg = Person()
                if len(station_ids) > 0:
                    person_msg.stationID = int(station_ids[inc])
                    person_msg.sensorID = station_ids[inc]
                person_msg.bodyParts = [None]*lenPoints

                for j in range(lenPoints):
                    person_msg.bodyParts[j] = Bodypart()

                for j in range(lenPoints):
                    person_msg.bodyParts[j].score = 0.8
                    person_msg.bodyParts[j].pixel.x = joints[j, 0]
                    person_msg.bodyParts[j].pixel.y = joints[j, 1]
                    person_msg.bodyParts[j].point.x =joints[j, 0] / 400-6 # TODO: @sm Please describe what these number stand for
                    person_msg.bodyParts[j].point.y =joints[j, 2] / 400-25
                    person_msg.bodyParts[j].point.z = -joints[j, 1] / 400
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

            img = cv2.hconcat(cropped_images)
            image_message = self.opencv_bridge.cv2_to_imgmsg(img, encoding="passthrough")
            self.publisher_crop[camera_id].publish(image_message)

        logy.log_fps("end_of_metrabs_loop")

if __name__ == '__main__':
    logy.basic_config(debug_level=logy.DEBUG, module_name="PE")

    rospy.init_node('metrabs', anonymous=True)
    run_spin_obj = PoseEstimator()
    logy.info("Pose Estimator is listening")
    rospy.spin()
