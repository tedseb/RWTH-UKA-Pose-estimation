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
from src.utils import draw_boxes, get_contours, get_masks
from src.contour_boxes import ContourBoxes
from src.weight_boxes import WeightBoxes
from src.config import *

def start_detection():
    img = cv2.imread('./data/image.jpg')
    masks = get_masks(img, COLOR_RANGES)
    cnts = get_contours(masks)

    box_contours = ContourBoxes(AREA_TRESHOLD)
    box_contours.add_contours(cnts)

    boxes = box_contours.merge_boxes_vertical()

    gewichte = WeightBoxes(boxes.copy(), masks.copy())
    gewichte.get_neighbour()
    barcode = gewichte.check_neighbour()
    print("barcode: ", barcode)

    draw_boxes(boxes, (255, 0, 0), img, 1)
    cv2.imshow('image1', img)
    cv2.waitKey(100000)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    start_detection()
