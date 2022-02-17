import os
import sys
import inspect
import cv2
#This lines are needed because the file should act like a top level file
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from src.yolo_model import YoloWeightModel
from src.utils import crop_weights_into_image
from src.dumbbell import Dumbbell
#pylint: disable=unused-wildcard-import
from src.config import *

MAX_FRAMES = 50

def start_box_detection_loop(max_frames, cap, model, pred, dumbbell : Dumbbell):
    for _ in range(max_frames):
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (1280, 720))

        dumbbell.add_masks(frame)
        dumbbell.crop_masks_into_image_and_display(frame)

        img = dumbbell.draw_boxes(frame)
        img = dumbbell.crop_weights_into_image(img)
        img = model.draw_pred_to_image(img, pred)
        
        cv2.imshow('frame', img)
        pressed_key = cv2.waitKey(16) & 0xFF
        if pressed_key == ord('q'):
            return False

    return True

def start_detection():
    cap_color = cv2.VideoCapture("./data/10_5Rund5_10L.avi")
    model = YoloWeightModel()

    while True:
        ret, frame = cap_color.read()
        if not ret:
            break

        frame = cv2.resize(frame, (1280, 720))
        output = model.inference(frame)
        pred = output.pred

        if model.is_valid_prediction(pred):
            left_box = model.get_left_box(pred)
            right_box = model.get_right_box(pred)
            dumbbell = Dumbbell(left_box, right_box, frame)
            ret1 = start_box_detection_loop(MAX_FRAMES, cap_color, model, pred, dumbbell)

            ret2, frame = cap_color.read()
            if not ret1 or not ret2:
                break

            dumbbell.add_masks(frame)
            dumbbell.crop_masks_into_image_and_display(frame)
            barcode_left, barcode_right = dumbbell.barcodes()
            img = dumbbell.draw_boxes(frame)
            img = dumbbell.crop_weights_into_image(img)
            img = model.draw_pred_to_image(img, pred)
            
            weight_left = 0
            for code in barcode_left:
                weight_left += COLOR_RANGES[code][1]

            weight_right = 0
            for code in barcode_right:
                weight_right += COLOR_RANGES[code][1]

            print("Barcode Left: ", barcode_left)
            print("Barcode Right: ", barcode_right)
            print(f"Weight Left: {weight_left}kg")
            print(f"Weight Right: {weight_right}kg")

            cv2.imshow('frame', img)
            pressed_key = cv2.waitKey(100000) & 0xFF
            if pressed_key == ord('q'):
                break
        else:
            img = crop_weights_into_image(img)
            cv2.imshow('frame', img)
            pressed_key = cv2.waitKey(16) & 0xFF
            if pressed_key == ord('q'):
                break

if __name__ == "__main__":
    start_detection()
