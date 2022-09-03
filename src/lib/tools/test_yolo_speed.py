#!/usr/bin/python3
import numpy as np
import torch
import time


#YOLO_PATH = '/home/trainerai/trainerai-core/src/AI/object_detection/yolov5'
YOLO_PATH = '/home/trainerai/trainerai-core/yolov5'
#MODEL_PATH = '/home/trainerai/trainerai-core/src/AI/object_detection/yolov5m.engine'
MODEL_PATH = '/home/trainerai/trainerai-core/yolov5/yolov5m.engine'
model = torch.hub.load(YOLO_PATH, 'custom', path=MODEL_PATH, source='local', force_reload=True)
img1 = np.empty([1280, 720, 3], dtype=np.uint8)
img2 = np.empty([1280, 720, 3], dtype=np.uint8)

results = model([img1, img2], size=640)
# print("#### wait 10 seconds ####")
# time.sleep(10)
time_stamp = time.time()
results = model([img1, img2], size=640)
time_elapsed = (time.time() - time_stamp) * 1000
print("TIME:", f"{time_elapsed}ms")
