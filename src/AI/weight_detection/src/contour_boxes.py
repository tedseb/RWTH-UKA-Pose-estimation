from typing import Callable
import numpy as np
import cv2

def union(a, b):
    # pylint: disable=invalid-name
    x = min(a[0], b[0])
    y = min(a[1], b[1])
    w = max(a[0]+a[2], b[0]+b[2]) - x
    h = max(a[1]+a[3], b[1]+b[3]) - y
    return (x, y, w, h)

def intersection(a, b):
    # pylint: disable=invalid-name
    x = max(a[0], b[0])
    y = max(a[1], b[1])
    w = min(a[0] + a[2], b[0] + b[2]) - x
    h = min(a[1] + a[3], b[1] + b[3]) - y
    if w < 0 or h < 0:
        return ()
    return (x, y, w, h)

def combine_boxes(boxes):
    combined_boxes = np.array(boxes, copy=True)

    pos_index = 0
    while pos_index < len(combined_boxes):
        intersections = True
        while intersections and len(combined_boxes) > 1 and pos_index < combined_boxes.shape[0]:
            box_a = combined_boxes[pos_index]
            iteration_list = np.delete(combined_boxes, pos_index, 0)
            for i, box_b in enumerate(iteration_list):
                if intersection(box_a, box_b):
                    new_box = union(box_a, box_b)
                    iteration_list[i] = new_box
                    combined_boxes = iteration_list
                    break
            else:
                intersections = False
        pos_index += 1

    return combined_boxes

class ContourBoxes:
    def __init__(self, area_treshold = 30):
        #self._merged_boxes = []
        self._boxes = []
        self._center_boxes = []
        self._rotation_filter = lambda r : (85 <= r <= 95) or r <= 5
        self._area_treshold = area_treshold

    def add_contours(self, contours):
        for contour in contours[0]:
            self.add_contour(contour)

    def add_contour(self, contour):
        (x_1, y_1), (w_1, h_1) , radius = cv2.minAreaRect(contour)
        if self._rotation_filter(radius):
            x_2, y_2, w_2, h_2 = cv2.boundingRect(contour)
            if w_2 * h_2 > self._area_treshold:
                self._boxes.append([x_2, y_2, w_2, h_2])
                self._center_boxes.append([int(x_1), int(y_1), w_1, h_1, abs(radius) ,0])

    def filter_radius_of_center_boxes(self, exp : Callable[[float], bool], area_treshold = 30):
        filtered_boxes = []
        for i, center_box in enumerate(self._center_boxes):
            rotation = float(center_box[4])
            if exp(rotation):
                if self._boxes[i][2] * self._boxes[i][3] > area_treshold:
                    filtered_boxes.append(self._boxes[i])
        return filtered_boxes

    def merge_boxes_vertical(self):
        merged_boxes = combine_boxes(self._boxes)

        pos_index = 0
        while pos_index < len(merged_boxes):
            intersections = True
            while intersections and len(merged_boxes) > 1 and pos_index < merged_boxes.shape[0]:
                box_a = merged_boxes[pos_index]
                iteration_list = np.delete(merged_boxes, pos_index, 0)
                for i, box_b in enumerate(iteration_list):
                    center_x_a = box_a[0] + box_a[2] / 2
                    left_x_b = box_b[0]
                    right_x_b = box_b[0] + box_b[2]
                    if left_x_b <= center_x_a <= right_x_b:
                        new_box = union(box_a, box_b)
                        iteration_list[i] = new_box
                        merged_boxes = iteration_list
                        break
                else:
                    intersections = False
            pos_index += 1

        return merged_boxes
