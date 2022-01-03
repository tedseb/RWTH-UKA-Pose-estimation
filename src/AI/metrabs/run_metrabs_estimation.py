#!/usr/bin/env python3
import os
import signal
import time
import threading
from multiprocessing import Lock
import numpy as np
import cv2
import rospy
import logy
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from backend.msg import ImageData, ChannelInfo
from backend.msg import Person, Persons, Bodypart, Bboxes
from std_msgs.msg import Bool
from gymy_tools import Queue

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf

THREAD_WAIT_TIME_MS = 20 #40 ms are the time beween two images at 25fps
AI_HEIGHT = 720
AI_WIDTH = 1280
AI_MODEL = 0 #0 = metrabs_multiperson_smpl, 1 = metrabs_rn34_y4

plt.switch_backend('TkAgg')

if AI_MODEL == 0:
    from tensorflow.python.keras.backend import set_session
    config = tf.compat.v1.ConfigProto()
    config.gpu_options.allow_growth = True # dynamically grow the memory used on the GPU
    config.log_device_placement = True # to log device placement (on which device the operation ran)
    sess = tf.compat.v1.Session(config=config)
    set_session(sess)
else:
    physical_devices = tf.config.list_physical_devices('GPU')
    if len(physical_devices) > 1:
        physical_devices = physical_devices[1:]

    tf.config.set_visible_devices(physical_devices,'GPU')
    tf.config.experimental.set_memory_growth(physical_devices[0], True)

if AI_MODEL == 0:
    CONFIG = {
        'intrinsics': [[1962, 0, 540], [0, 1969, 960], [0, 0, 1]], # [[3324, 0, 1311], [0, 1803, 707], [0, 0, 1]]
        'model_path': '/home/trainerai/trainerai-core/src/AI/metrabs/models/metrabs_multiperson_smpl'
    }
elif AI_MODEL == 1:
    CONFIG = {
        'intrinsics': [[1962, 0, 540], [0, 1969, 960], [0, 0, 1]],
        'model_path': '/home/trainerai/trainerai-core/src/AI/metrabs/models/metrabs_rn34_y4'
    }

class PoseEstimator():
    def __init__(self):
        self.ready_signal = rospy.Publisher('/signals/metrabs_ready', Bool, queue_size=2)
        rospy.Subscriber('bboxes', Bboxes, self.callback_regress, queue_size=10)
        rospy.Subscriber('/channel_info', ChannelInfo, self.handle_new_channel)

        self._intrinsics = tf.constant([CONFIG.get("intrinsics")], dtype=tf.float32)
        self._publisher = rospy.Publisher('personsJS', Persons, queue_size=10)
        self._publisher_crop = {}
        self._subscriber_image = {}
        self._opencv_bridge = CvBridge()
        self._send_cropped = True

        self._image_queues = {}
        self._debug_time_queues = {}
        self._box_queues = {}
        self._thread_lock = Lock()

        #img = cv2.imread('../data/ted_image.jpg')
        #box = [680.0, 180.0, 180.0, 490.0]
        self.model = tf.saved_model.load(CONFIG["model_path"])
        self._fake_image = np.empty([AI_HEIGHT, AI_WIDTH, 3], dtype=np.uint8)
        self._fake_person_boxes = [np.array([100, 100, 100, 100], np.float32)]
        self._fake_image = cv2.imread('/home/trainerai/trainerai-core/src/AI/metrabs/image.jpg')
        self._fake_person_boxes = [np.array([680.0, 180.0, 180.0, 490.0], np.float32)]
        self.start_ai([], np.stack([self._fake_image]), [self._fake_person_boxes], set())

        self._is_active = True
        self._ai_thread = threading.Thread(target=self.thread_handler)
        self._ai_thread.start()

    def __del__(self):
        self._is_active = False
        self._ai_thread.join(timeout=1)

    def shutdown(self, signum, frame):
        self._is_active = False
        self._ai_thread.join(timeout=1)
        rospy.signal_shutdown("Shutdown")
        #print("shutdown")

    @logy.catch_ros
    def handle_new_channel(self, channel_info: ChannelInfo):
        with self._thread_lock:
            if channel_info.is_active:
                logy.debug(f"New Channel: {channel_info.channel_name}")
                if channel_info.cam_id not in self._subscriber_image:
                    sub = rospy.Subscriber(channel_info.channel_name, ImageData, self.callback_setImage)
                    self._subscriber_image[channel_info.cam_id] = sub
                if channel_info.cam_id not in self._publisher_crop and self._send_cropped:
                    pub = rospy.Publisher(f'{channel_info.channel_name}_cropped', Image , queue_size=2)
                    self._publisher_crop[channel_info.cam_id] = pub
                if channel_info.cam_id not in self._image_queues:
                    self._image_queues[channel_info.cam_id] = Queue(15)
                if channel_info.cam_id not in self._debug_time_queues:
                    self._debug_time_queues[channel_info.cam_id] = Queue(20)
                if channel_info.cam_id not in self._box_queues:
                    self._box_queues[channel_info.cam_id] = Queue(5)
            else:
                if channel_info.cam_id in self._subscriber_image:
                    self._subscriber_image[channel_info.cam_id].unregister()
                    del self._subscriber_image[channel_info.cam_id]
                if channel_info.cam_id in self._publisher_crop and self._send_cropped:
                    self._publisher_crop[channel_info.cam_id].unregister()
                    del self._publisher_crop[channel_info.cam_id]
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
            #logy.debug(f"Received image. Debug frame {msg.debug_id}", tag="debug_frame")
            time_ms = time.time() * 1000
            self._debug_time_queues[camera_id].put((time_ms, msg.debug_id))

        if camera_id in self._image_queues:
            self._image_queues[camera_id].put(msg)
            #logy.log_fps("metraps_image_fps", 50)

    def get_next_image_in_queue(self, box_frame_number: int, camera_id: int):
        queue = self._image_queues.get(camera_id)
        if queue is None or queue.empty():
            return None

        bbox_num = box_frame_number
        next_img_num = queue[0].frame_num
        if next_img_num > box_frame_number:
            logy.debug(f"Image not in queu. Img={next_img_num}, box={bbox_num}")

        while next_img_num <= bbox_num:
            img_data = queue.get()
            if next_img_num == bbox_num:
                return img_data
            if queue.empty():
                break
            next_img_num = queue[0].frame_num

        logy.debug(f"2: next = {next_img_num}, box = {bbox_num}")
        return None

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
                logy.log_fps("new_box_received", 100)

    @logy.catch_thread_and_restart
    def thread_handler(self):
        logy.debug("Metrabs Thread is active")
        thread_wait_time = THREAD_WAIT_TIME_MS / 1000.0

        while(self._is_active):
            boxes = []
            images = []
            data = []
            with self._thread_lock:
                fake_images = set()
                for camera_id, queue in self._box_queues.items():
                    if queue.empty():
                        fake_images.add(camera_id)
                        data.append((camera_id, None, None))
                        images.append(self._fake_image)
                        boxes.append(self._fake_person_boxes)
                        continue

                    box = queue.get()
                    box_np = np.array(box.data).reshape(-1, 4)
                    if box_np.size == 0:
                        fake_images.add(camera_id)
                        data.append((camera_id, None, None))
                        images.append(self._fake_image)
                        boxes.append(self._fake_person_boxes)
                        continue

                    img_data = self.get_next_image_in_queue(box.frame_num, camera_id)
                    if img_data is None:
                        fake_images.add(camera_id)
                        data.append((camera_id, None, None))
                        images.append(self._fake_image)
                        boxes.append(self._fake_person_boxes)
                        data.append((camera_id, None, None))
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
                        box_np[:, 0] *= w_factor
                        box_np[:, 1] *= h_factor
                        box_np[:, 2] *= w_factor
                        box_np[:, 3] *= h_factor

                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    images.append(image)
                    boxes.append(box_np.astype(np.float32))
                    data.append((camera_id, box.stationID, last_image.header))

            if not data:
                time.sleep(thread_wait_time)
                continue

            images = np.stack(images)
            if images.shape[0] != len(self._box_queues):
                logy.warn(f"only {images.shape[0]} boxes but {len(self._box_queues)}. Queue: {fake_images}")
            self.start_ai(data, images, boxes, fake_images)

    def start_ai(self, data, images, boxes, fake_images):
        ragged_boxes = tf.ragged.constant(boxes, ragged_rank=1)
        logy.debug_throttle(f"{images.shape[0]}", 1000)

        with logy.TraceTime("matrabs_multi_image"):
            if AI_MODEL == 0:
                pred_output_list = self.model.predict_multi_image(images, self._intrinsics, ragged_boxes)
            else:
                pred_output_list = self.model.estimate_poses_batched(images, boxes=ragged_boxes, intrinsic_matrix=self._intrinsics,  skeleton='smpl_24')["poses3d"]
        pred_output_list = pred_output_list.numpy()


        for img_index, info in enumerate(data):
            camera_id = info[0]
            if camera_id in fake_images:
                continue

            station_ids = info[1]
            image_boxes = boxes[img_index]
            image = images[img_index]

            inc = 0
            msg = Persons()
            msg.header = info[2]
            msg.persons = list()
            cropped_images = []

            if len(pred_output_list[img_index]) == 0:
                rospy.logerr_throttle(5, "Station is active but Pose Estimator could not detect people.")
                continue

            for prediction_index, detection in enumerate(pred_output_list[img_index]):

                joints = detection
                bb = image_boxes[prediction_index]
                cropped_image = image[int(bb[1]):int(bb[1]+bb[3]),int(bb[0]):int(bb[0]+bb[2])]
                cropped_images.append(cropped_image)

                lenPoints=len(joints)       # TODO: use fixed number of joints to save calculation time
                person_msg = Person()
                if len(station_ids) > 0:
                    person_msg.stationID = int(station_ids[inc])
                    person_msg.sensorID = station_ids[inc]
                person_msg.bodyParts = [None] * lenPoints

                for j in range(lenPoints):
                    person_msg.bodyParts[j] = Bodypart()

                for j in range(lenPoints):
                    person_msg.bodyParts[j].score = 0.8
                    person_msg.bodyParts[j].pixel.x = joints[j, 0]
                    person_msg.bodyParts[j].pixel.y = joints[j, 1]
                    person_msg.bodyParts[j].point.x = joints[j, 0] / 400-6 # TODO: @sm Please describe what these number stand for
                    person_msg.bodyParts[j].point.y = joints[j, 2] / 400-25
                    person_msg.bodyParts[j].point.z = -joints[j, 1] / 400
                inc += 1
                msg.persons.append(person_msg)

            self._publisher.publish(msg)

            if len(cropped_images) > 1:
                max_h = 0
                for img in cropped_images:
                    max_h = max(max_h, img.shape[0])


                for i, img in enumerate(cropped_images):
                    height_difference = max_h - img.shape[0]
                    cropped_images[i] = cv2.copyMakeBorder(img, height_difference, 0, 0, 0, cv2.BORDER_CONSTANT | cv2.BORDER_ISOLATED, (0, 0, 0))

            # Concatenate images and convert them to ROS image format to display them later in rviz
            img = cv2.hconcat(cropped_images)
            image_message = self._opencv_bridge.cv2_to_imgmsg(img, encoding="passthrough")
            pub = self._publisher_crop.get(camera_id)
            if pub is not None:
                pub.publish(image_message)
                logy.log_fps("Publish")

if __name__ == '__main__':
    logy.basic_config(debug_level=logy.DEBUG, module_name="PE")
    rospy.init_node('metrabs', anonymous=True)
    run_spin_obj = PoseEstimator()
    signal.signal(signal.SIGTERM, run_spin_obj.shutdown)
    signal.signal(signal.SIGINT, run_spin_obj.shutdown)
    logy.info("Pose Estimator is listening")
    run_spin_obj.ready_signal.publish(True)
    rospy.spin()
