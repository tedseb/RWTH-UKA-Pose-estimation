#!/usr/bin/python3
import sys
import argparse
from ctypes import resize
import threading
import collections
import time
from dataclasses import dataclass
from multiprocessing import Lock
from typing import List, Dict, Tuple
import cv2
import numpy as np
import rospy
import yaml
import torch
import logy
import signal
from gymy_tools import Queue
from backend.msg import ImageData, ChannelInfo, LabelsCameraID, Bboxes
from sensor_msgs.msg import Image
from scheduler import BoxChecker

YOLO_PATH = '/home/trainerai/trainerai-core/src/AI/object_detection/yolov5'
MODEL_PATH = '/home/trainerai/trainerai-core/src/AI/object_detection/yolov5s.pt'
IMAGE_QUEUE_LEN = 2
THREAD_WAIT_TIME_MS = 5
AI_HEIGHT = 720
AI_WIDTH = 1280
USE_STATION_FRAMES = False

@dataclass
class YoloData:
    labels: List[int] = None
    confs: List[float] = None
    boxes: List[List[int]] = None
    render_img = None


    #raise ServiceExit

class ObjectDetectionPipeline:
    def __init__(self, threshold=0.5, device="cpu", renderer=False, check_station=False):
        self._box_checker = BoxChecker()
        self._check_station = check_station
        self._model = torch.hub.load(YOLO_PATH, 'custom', path=MODEL_PATH, source='local') # .eval().to(device)
        self._threshold = threshold
        self._renderer = renderer
        self._publisher_boxes = rospy.Publisher('bboxes', Bboxes , queue_size=10)
        self._publisher_labels = rospy.Publisher('labels', LabelsCameraID , queue_size=10)
        self._subscriber_image = {}
        self._publisher_img_yolo = {}
        self._image_queues : Dict[Queue] = {}

        rospy.Subscriber('/channel_info', ChannelInfo, self.handle_new_channel)
        logy.info("Object Detection is listening")

        self._image_queues_lock = Lock()
        self._is_active = True
        self._detection_thread = threading.Thread(target=self._object_detector_loop)
        self._detection_thread.start()

    def __del__(self):
        self._is_active = False
        self._detection_thread.join(timeout=1)

    def shutdown(self, signum, frame):
        self._is_active = False
        self._detection_thread.join(timeout=1)
        rospy.signal_shutdown("Shutdown")
        #print("shutdown")

    @logy.catch_ros
    def callback_set_image(self, msg: ImageData):
        camera_id = int(msg.image.header.frame_id[3:])
        # if msg.is_debug:
        #     time_ms = time.time() * 1000
        #     self._debug_time_queues[camera_id].put((time_ms, msg.debug_id))
        if camera_id in self._image_queues:
            self._image_queues[camera_id].put(msg)

    @logy.catch_ros
    def handle_new_channel(self, channel_info: ChannelInfo):
        with self._image_queues_lock:
            if channel_info.is_active:
                logy.debug(f"New Channel: {channel_info.channel_name}")
                if channel_info.cam_id not in self._subscriber_image:
                    sub = rospy.Subscriber(channel_info.channel_name, ImageData, self.callback_set_image)
                    self._subscriber_image[channel_info.cam_id] = sub
                if channel_info.cam_id not in self._publisher_img_yolo and self._renderer:
                    pub = rospy.Publisher(f'{channel_info.channel_name}_yolo', Image , queue_size=2)
                    self._publisher_img_yolo[channel_info.cam_id] = pub
                if channel_info.cam_id not in self._image_queues:
                    self._image_queues[channel_info.cam_id] = Queue(IMAGE_QUEUE_LEN)
            else:
                if channel_info.cam_id in self._subscriber_image:
                    self._subscriber_image[channel_info.cam_id].unregister()
                    del self._subscriber_image[channel_info.cam_id]
                if channel_info.cam_id in self._publisher_img_yolo and self._renderer:
                    self._publisher_img_yolo[channel_info.cam_id].unregister()
                    del self._publisher_img_yolo[channel_info.cam_id]
                if channel_info.cam_id in self._image_queues:
                    del self._image_queues[channel_info.cam_id]

    @logy.catch_thread_and_restart
    def _object_detector_loop(self):
        '''
        This is a thread function which pull images from the queues.
        The image is reshaped and sent to the yolo detection. The results are published as boxes.

        '''
        logy.debug_throttle("Object Detection thread is active", 1000)
        thread_wait_time = THREAD_WAIT_TIME_MS / 1000.0
        while(self._is_active):
            images = []
            camera_ids = []
            image_data = []
            resize_factors = []
            with self._image_queues_lock:

                for camera_id, queue in self._image_queues.items():
                    if queue.empty():
                        continue

                    img_data = queue.get()
                    if img_data.is_debug:
                        logy.debug(f"Received image. Debug frame {img_data.debug_id}", tag="debug_frame")

                    img_msg = img_data.image
                    shape = img_msg.height, img_msg.width, 3                            #(480, 640, 3) --> (y,x,3)
                    img = np.frombuffer(img_msg.data, dtype=np.uint8)
                    img_original_bgr = img.reshape(shape)
                    img_original_bgr = cv2.resize(img_original_bgr, (AI_WIDTH, AI_HEIGHT))
                    w_factor = img_msg.width / AI_WIDTH
                    h_factor = img_msg.height / AI_HEIGHT
                    resize_factors.append((w_factor, h_factor))

                    images.append(img_original_bgr)
                    camera_ids.append(camera_id)
                    image_data.append(img_data)

                if not images:
                    time.sleep(thread_wait_time)
                    continue

                station_boxes_list = []
                if not USE_STATION_FRAMES:
                    yolo_data_results = self.detect_objects(images, resize_factors)
                    if len(yolo_data_results) == 0:
                        continue
                    for i, yolo_data in enumerate(yolo_data_results):
                        if yolo_data is None:
                            continue
                        station_boxes = self._get_person_boxes_in_station(yolo_data, camera_ids[i], shape[1], shape[0]) #[x, y, w, h]
                        station_boxes_list.append(station_boxes)
                        self._publish_render_image(yolo_data.render_img, img_msg.header.frame_id, camera_ids[i])
                        self._publish_labels(yolo_data.labels, image_data[i])
                else:
                    for camera_id in camera_ids:
                        station_boxes = self._box_checker.get_camera_station_frames(camera_id)
                        for box in station_boxes.values():
                            box[2] -= box[0]
                            box[3] -= box[1]
                        station_boxes_list.append(station_boxes)

                for i, station_boxes in enumerate(station_boxes_list):
                    logy.log_fps("publish_boxes")
                    self._publish_boxes(station_boxes, image_data[i], camera_ids[i])
                    #logy.log_fps("object_detection_fps")

    #@logy.trace_time("detect_objects")
    def detect_objects(self, imgs : List, resize_factors : Tuple) -> List[YoloData]:
        '''
        This function uses the Yolo object detector. It predicts BBOX with label and confidence values.
        Returns:
            List[YoloData]: Yolo Predictions for each image. YoloData is None if there is no Prediction for this index.
        '''

        with logy.TraceTime("time_yolo_model"):
            results = self._model(imgs, size=640)

        yolo_data_results = []
        for i in range(len(imgs)):
            yolo_data = YoloData()
            if self._renderer:
                results.render()
                yolo_data.render_img = results.imgs[i]

            result_np = results.xyxy[i].cpu().detach().numpy() #x1, y1, x2, y2
            if result_np.size == 0:
                #logy.warn_throttle("No Results in Object Detection", 1000)
                #yolo_data_results.append(None)
                continue

            result_np[:, 0] *= resize_factors[i][0]
            result_np[:, 1] *= resize_factors[i][1]
            result_np[:, 2] *= resize_factors[i][0]
            result_np[:, 3] *= resize_factors[i][1]
            sorted(result_np, key=lambda entry : (entry[2] - entry[0]) * (entry[3] - entry[1])) # Sort list by area

            yolo_data.labels = result_np[:,5]
            yolo_data.confs = result_np[:,4]
            yolo_data.boxes = np.array(result_np[:,:4])
            yolo_data_results.append(yolo_data)
        logy.log_var("yolo_batch_size", len(yolo_data_results), period=25)
        return yolo_data_results

    def _get_person_boxes_in_station(self, yolo_data : YoloData, camera_id : int, img_w : int, img_h : int):
        '''Function for checking labels and assigning BBOX to stations'''

        fake_station_id = 1000
        station_boxes = {}

        for index, label in enumerate(yolo_data.labels):
            box = yolo_data.boxes[index]
            if label != 0:
                continue

            if self._check_station:
                station_id = self._box_checker.check_box_in_station(box, camera_id)
                if station_id < 0:
                    continue
            else:
                station_id = fake_station_id
                fake_station_id += 1

            if station_id in station_boxes or yolo_data.confs[index] < self._threshold:
                continue

            extra_width_w = 0 #img_w * 0.03
            extra_width_h = 0 #img_h * 0.03
            x = box[0] - extra_width_w
            y = box[1] - extra_width_h
            w = box[2] + extra_width_w - x
            h = box[3] + 2 * extra_width_h - y
            x = max(x, 0)
            y = max(y, 0)
            w = min(w, img_w - x)
            h = min(h, img_h - y)
            station_boxes[station_id] = [x,y,w,h]
        return station_boxes

    def _publish_render_image(self, img, frame_id : str, camera_id : int):
        if not self._renderer or img is None:
            return

        msg_render_image = Image()
        msg_render_image.header.stamp = rospy.Time.now()
        msg_render_image.header.frame_id = frame_id
        msg_render_image.encoding = "bgr8"
        msg_render_image.data = np.array(img, dtype=np.uint8).tobytes()
        msg_render_image.height, msg_render_image.width = img.shape[:-1]
        msg_render_image.step = img.shape[-1]*img.shape[0]
        self._publisher_img_yolo[camera_id].publish(msg_render_image)

    def _publish_boxes(self, station_boxes, old_img_data, camera_id : int):
        box_list_1d = []
        if not station_boxes:
            return

        station_boxes = collections.OrderedDict(sorted(station_boxes.items()))
        for box in station_boxes.values():
            box_list_1d.extend(box)
        box_msg  = Bboxes()
        box_msg.header.stamp = old_img_data.image.header.stamp #Will be important for data fusion: Use current time or older stamp from CameraNode
        box_msg.header.frame_id = old_img_data.image.header.frame_id
        box_msg.frame_num = old_img_data.frame_num
        box_msg.data = box_list_1d
        box_msg.stationID = list(station_boxes.keys())
        box_msg.sensorID = [camera_id for i in range(len(station_boxes))]
        box_msg.is_debug = False
        if old_img_data.is_debug:
            box_msg.is_debug = True
            box_msg.debug_id = old_img_data.debug_id
            logy.debug(f"Publish bboxes. Debug frame {old_img_data.debug_id}", tag="debug_frame")
        self._publisher_boxes.publish(box_msg)

    def _publish_labels(self, labels, old_img_data):
        label_msg  = LabelsCameraID()
        label_msg.header.stamp = old_img_data.image.header.stamp #Will be important for data fusion: Use current time or older stamp from CameraNode
        label_msg.header.frame_id = old_img_data.image.header.frame_id  #From which camera
        label_msg.sensorID = old_img_data.image.header.frame_id
        label_msg.data = list(labels)
        self._publisher_labels.publish(label_msg)

    def spin(self):
        '''We enter in a loop and wait for exit whenever `Ctrl + C` is pressed'''
        rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--render", help="Draw and publish yolo images", action="store_true")

    arg_count = len(sys.argv)
    #print(sys.argv)
    last_arg = sys.argv[arg_count - 1]
    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()

    logy.basic_config(debug_level=logy.DEBUG, module_name="OD")
    rospy.init_node('object_detection', anonymous=True)
    rospy.set_param('param_server', yaml.dump({0: {}}))
    obj_detect = ObjectDetectionPipeline(device="cuda", threshold=0.5, renderer=args.render, check_station=True)
    signal.signal(signal.SIGTERM, obj_detect.shutdown)
    signal.signal(signal.SIGINT, obj_detect.shutdown)
    obj_detect.spin()
