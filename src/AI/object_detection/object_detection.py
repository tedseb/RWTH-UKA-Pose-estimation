#!/usr/bin/python3
from dataclasses import dataclass
import numpy as np
from typing import List
from backend.msg import ImageData
from backend.msg import Bboxes
from backend.msg import LabelsCameraID
from sensor_msgs.msg import Image
from scheduler import BoxChecker
import time
import rospy
import yaml
import torch
import logy


YOLO_PATH = '/home/trainerai/trainerai-core/src/AI/object_detection/yolov5'
MODEL_PATH = '/home/trainerai/trainerai-core/src/AI/object_detection/yolov5s.pt'

@dataclass
class YoloData:
    labels: List[int] = None
    confs: List[float] = None
    boxes: List[List[int]] = None
    render_img = None

class ObjectDetectionPipeline:
    def __init__(self, threshold=0.5, device="cpu", renderer=False, check_station=False):
        self._box_checker = BoxChecker()
        self._check_station = check_station
        self._model = torch.hub.load(YOLO_PATH, 'custom', path=MODEL_PATH, source='local') # .eval().to(device)
        self._threshold = threshold
        self._renderer = renderer
        self.publisher_boxes = rospy.Publisher('bboxes', Bboxes , queue_size=2)
        self.publisher_labels = rospy.Publisher('labels', LabelsCameraID , queue_size=2)
        if self._renderer:
            self.publisher_img = rospy.Publisher('imageYOLO', Image , queue_size=2)

        rospy.Subscriber('image', ImageData, self._object_detector_loop)
        logy.info("Object Detection is listening")

    def _object_detector_loop(self, img_data: ImageData):
        '''
        This is a callback function which receives the message from the subscriber.
        The message contains an image from the camera, this is reshaped,
        sent to the function obj_detectYolo() and then the result (BBOX) is published.
        '''
        if img_data.is_debug:
            logy.debug(f"Received image. Debug frame {img_data.debug_id}", tag="debug_frame")

        img_msg = img_data.image
        shape = img_msg.height, img_msg.width, 3                            #(480, 640, 3) --> (y,x,3)
        img = np.frombuffer(img_msg.data, dtype=np.uint8)
        img_original_bgr = img.reshape(shape)
        camera_id = int(img_msg.header.frame_id[3:])

        yolo_data = self.detect_objects(img_original_bgr)
        if yolo_data is None:
            return

        with logy.TraceTime("time_trace_test2"):
            station_boxes = self._get_person_boxes_in_station(yolo_data, camera_id)

            self._publish_boxes(station_boxes, img_data, camera_id)
            self._publish_render_image(yolo_data.render_img, img_msg.header.frame_id)
            self._publish_labels(yolo_data.labels, img_data)
            logy.log_fps("object_detection_fps")

    @logy.catch_ros
    def detect_objects(self, img) -> YoloData:
        '''This function uses the Yolo object detector. It predicts BBOX with label and confidence values.'''
        img_tens = img
        tmpTime = time.time()
        results = self._model(img_tens, size=640)
        fps = int(1/(time.time()-tmpTime))
        logy.log_var("yolo_fps", fps, period=25, smoothing=0.9)

        yolo_data = YoloData()
        if self._renderer:
            results.render()  # updates results.imgs with boxes and labels
            yolo_data.render_img = results.imgs[0]

        result_np = results.xyxy[0].cpu().detach().numpy() # BBox is in x1,y1,x2,y2
        if result_np.size == 0:
            rospy.logerr_throttle(5, "Ojbect detection is currently failing. If you see this message repeatedly, there is something wrong...")
            return None

        sorted(result_np, key=lambda entry : (entry[2] - entry[0]) * (entry[3] - entry[1])) # Sort list by area

        yolo_data.labels = result_np[:,5]
        yolo_data.confs = result_np[:,4]
        yolo_data.boxes = result_np[:,:4]
        return yolo_data

    def _get_person_boxes_in_station(self, yolo_data : YoloData, camera_id : int):
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

            extra_width = (box[2] - box[0]) * 0.1
            x = box[0] - extra_width
            y = box[1] - extra_width
            w = box[2] + extra_width - x
            h = box[3] + 2 * extra_width - y
            station_boxes[station_id] = [x,y,w,h]
        return station_boxes

    def _publish_render_image(self, img, frame_id : str):
        if not self._renderer or img is None:
            return

        msg_render_image = Image()
        msg_render_image.header.stamp = rospy.Time.now()
        msg_render_image.header.frame_id = frame_id
        msg_render_image.encoding = "bgr8"
        msg_render_image.data = np.array(img, dtype=np.uint8).tobytes()
        msg_render_image.height, msg_render_image.width = img.shape[:-1]
        msg_render_image.step = img.shape[-1]*img.shape[0]
        self.publisher_img.publish(msg_render_image)

    def _publish_boxes(self, station_boxes, old_img_data, camera_id : int):
        box_list_1d = []
        for box in station_boxes.values():
            box_list_1d.extend(box)
        box_msg  = Bboxes()
        box_msg.header.stamp = old_img_data.image.header.stamp #Will be important for data fusion: Use current time or older stamp from CameraNode
        box_msg.header.frame_id = old_img_data.image.header.frame_id
        box_msg.data = box_list_1d
        box_msg.stationID = list(station_boxes.keys())
        box_msg.sensorID = [camera_id for i in range(len(station_boxes))]
        box_msg.is_debug = False
        if old_img_data.is_debug:
            box_msg.is_debug = True
            box_msg.debug_id = old_img_data.debug_id
            logy.debug(f"Publish bboxes. Debug frame {old_img_data.debug_id}", tag="debug_frame")
        self.publisher_boxes.publish(box_msg)

    def _publish_labels(self, labels, old_img_data):
        label_msg  = LabelsCameraID()
        label_msg.header.stamp = old_img_data.image.header.stamp #Will be important for data fusion: Use current time or older stamp from CameraNode
        label_msg.header.frame_id = old_img_data.image.header.frame_id  #From which camera
        label_msg.sensorID = old_img_data.image.header.frame_id
        label_msg.data = list(labels)
        self.publisher_labels.publish(label_msg)

    def spin(self):
        '''We enter in a loop and wait for exit whenever `Ctrl + C` is pressed'''
        rospy.spin()

if __name__ == '__main__':
    logy.basic_config(debug_level=logy.DEBUG, module_name="OD")
    rospy.init_node('object_detection', anonymous=True)
    rospy.set_param('param_server', yaml.dump({0: {}}))
    obj_detect = ObjectDetectionPipeline(device="cuda", threshold=0.5, renderer=True, check_station=True)
    obj_detect.spin()
