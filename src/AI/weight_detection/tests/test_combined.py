import os
import sys
import inspect
import cv2
#This lines are needed because the file should act like a top level file
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
#pylint: disable=wrong-import-position
#pylint: disable=unused-wildcard-import
from src.yolo_model import YoloWeightModel
from src.utils import draw_boxes, get_contours, get_masks, crop_weights_into_image
from src.contour_boxes import ContourBoxes
from src.weight_boxes import WeightBoxes
from src.config import *

def compute_barcode(img):
    masks = get_masks(img, COLOR_RANGES)
    cnts = get_contours(masks)

    box_contours = ContourBoxes(AREA_TRESHOLD)
    box_contours.add_contours(cnts)

    boxes = box_contours.merge_boxes_vertical()
    draw_boxes(boxes, (255, 0, 0), img, 1)

    gewichte = WeightBoxes(boxes.copy(), masks.copy())
    gewichte.get_neighbour()
    barcode = gewichte.check_neighbour()
    return barcode

def start_detection_image():
    model = YoloWeightModel()

    frame = cv2.imread('./data/image.jpg')
    output = model.inference(frame)
    pred = output.pred

    if not model.is_valid_prediction(pred):
        return

    left_box = model.get_left_box(pred)
    right_box = model.get_right_box(pred)

    left_img = frame[left_box[1]:left_box[3], left_box[0]:left_box[2]].copy()
    right_img = frame[right_box[1]:right_box[3], right_box[0]:right_box[2]].copy()


    barcode_left = compute_barcode(left_img)
    barcode_right = compute_barcode(right_img)

    print("Barcode Left: ", barcode_left)
    print("Barcode Right: ", barcode_right)

    img = model.draw_pred_to_image(frame, pred)
    img = crop_weights_into_image(img, left_img, right_img)

    cv2.imshow("image", img)
    cv2.waitKey(10000)

def start_detection_video():
    cap_color = cv2.VideoCapture("./data/10_5Rund5_10L.avi")
    model = YoloWeightModel()

    while True:
        ret, frame = cap_color.read()
        if not ret:
            break
        frame = cv2.resize(frame, (1280, 720))
        output = model.inference(frame)
        pred = output.pred
        img = model.draw_pred_to_image(frame, pred)

        if model.is_valid_prediction(pred):
            left_box = model.get_left_box(pred)
            right_box = model.get_right_box(pred)

            left_img = frame[left_box[1]:left_box[3], left_box[0]:left_box[2]]
            right_img = frame[right_box[1]:right_box[3], right_box[0]:right_box[2]]

            barcode_left = compute_barcode(left_img)
            barcode_right = compute_barcode(right_img)

            print("Barcode Left: ", barcode_left)
            print("Barcode Right: ", barcode_right)

            img = crop_weights_into_image(img, left_img, right_img)
        else:
            img = crop_weights_into_image(img)

        cv2.imshow('frame', img)
        pressed_key = cv2.waitKey(16) & 0xFF
        if pressed_key == ord('q'):
            break

if __name__ == "__main__":
    start_detection_video()
