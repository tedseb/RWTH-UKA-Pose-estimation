import os
import sys
import inspect
import cv2
#This lines are needed because the file should act like a top level file
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
#pylint: disable=wrong-import-position
from src.yolo_model import YoloWeightModel
from src.utils import crop_weights_into_image

# Was ist Model fuse autoshape? (Wird in yolo benutzt)
# Warum mehrere Predictions möglich im Tensor?
# [tensor([[868.00000, 190.50000, 928.00000, 325.50000,   0.93994,   1.00000],
#        [432.25000, 174.12500, 472.25000, 315.00000,   0.93311,   0.00000]], device='cuda:0')]

def test_image():
    model = YoloWeightModel()

    frame = cv2.imread('./data/image.jpg')
    output = model.inference(frame)
    pred = output.pred
    img = model.draw_pred_to_image(frame, pred)

    if not model.is_valid_prediction(pred):
        return

    left_box = model.get_left_box(pred)
    right_box = model.get_right_box(pred)

    left_img = frame[left_box[1]:left_box[3], left_box[0]:left_box[2]]
    right_img = frame[right_box[1]:right_box[3], right_box[0]:right_box[2]]
    img = cv2.resize(img, (1280, 720))
    img = crop_weights_into_image(img, left_img, right_img)

    cv2.imshow("image", img)
    cv2.waitKey(10000)

def test_video():
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
            img = crop_weights_into_image(img, left_img, right_img)
        else:
            img = crop_weights_into_image(img)

        cv2.imshow('frame', img)
        pressed_key = cv2.waitKey(16) & 0xFF
        if pressed_key == ord('q'):
            break

if __name__ == "__main__":
    test_video()
