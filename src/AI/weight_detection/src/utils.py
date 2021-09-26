from typing import List, Tuple
import warnings
import cv2
import numpy as np

def get_masks(img, color_ranges : List[Tuple]):
    mask_list = []

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    for color_range in color_ranges:
        lower_color = np.array(color_range[2])
        upper_color = np.array(color_range[3])
        mask = cv2.inRange(hsv, lower_color, upper_color)
        mask_list.append(mask)

    return mask_list

def get_contours(mask_list):
    contour_array= []
    for mask in mask_list:
        contour_array.append(cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2])
    return contour_array

def draw_boxes(boxes, colour ,img, width):
    for box in boxes:
        cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[0] + box[2]), int(box[1] + box[3])), colour, width)

def crop_weights_into_image(img, left_weight_img = None, right_weight_img = None, copy = False):
    if copy:
        img = img.copy()
    img_h = img.shape[0]
    img_w = img.shape[1]

    if img_h % 2 != 0:
        raise RuntimeError("Height must be multiple of 2")

    stack_width = int(img_h / 2)
    stack_img = np.zeros((img_h, stack_width, 3),dtype=np.uint8)
    img = np.hstack((img, stack_img))

    if left_weight_img is None or left_weight_img is None:
        return img

    scales_left = [stack_width / left_weight_img.shape[0], stack_width / left_weight_img.shape[1]]
    scale_left = min(scales_left)
    left_img = cv2.resize(left_weight_img, None, fx=scale_left, fy=scale_left)

    offset = int((stack_width - left_img.shape[1]) / 2)
    offset = img_w + max(0, offset)
    img[0:left_img.shape[0], offset:offset+left_img.shape[1]] = left_img

    scales_right = [stack_width / right_weight_img.shape[0], stack_width / right_weight_img.shape[1]]
    scale_right = min(scales_right)
    right_img = cv2.resize(right_weight_img, None, fx=scale_right, fy=scale_right)

    offset = int((stack_width - right_img.shape[1]) / 2)
    offset = img_w + max(0, offset)
    img[stack_width:stack_width + right_img.shape[0], offset:offset+right_img.shape[1]] = right_img

    return img

def crop_masks_into_image(left_weight_img, right_weight_img, masks_left, masks_right, colors):
    #colors = [(0, 255, 255), (0, 0, 255), (255, 0, 0), (0, 255, 0)]
    mask_len = len(masks_left)
    if mask_len != len(colors) or mask_len != len(masks_right):
        warnings.warn("Arguments must be 'len(masks_left) == len(masks_right) == len(names)'")
        return

    stack_height = 360
    scale_left = stack_height / left_weight_img.shape[0]
    stack_width = int(left_weight_img.shape[1] * scale_left)

    stacked_img_left = cv2.resize(left_weight_img, None, fx=scale_left, fy=scale_left)
    for i, mask in enumerate(masks_left):
        resized_mask = cv2.resize(mask, None, fx=scale_left, fy=scale_left)
        resized_mask = cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2RGB)
        resized_mask = cv2.rectangle(resized_mask, (0, 0), (stack_width, stack_height), colors[i], 6)
        stacked_img_left = np.hstack((stacked_img_left, resized_mask))

    #scales_right = [stack_width / right_weight_img.shape[0], stack_width / right_weight_img.shape[1]]
    scale_right = stack_height / right_weight_img.shape[0]
    stack_width = int(right_weight_img.shape[1] * scale_right)

    stacked_img_right = cv2.resize(right_weight_img, None, fx=scale_right, fy=scale_right)
    for i, mask in enumerate(masks_right):
        resized_mask = cv2.resize(mask, None, fx=scale_right, fy=scale_right)
        resized_mask = cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2RGB)
        resized_mask = cv2.rectangle(resized_mask, (0, 0), (stack_width, stack_height), colors[i], 6)
        stacked_img_right = np.hstack((stacked_img_right, resized_mask))

    left_w = stacked_img_left.shape[1]
    right_w = stacked_img_right.shape[1]
    if left_w < right_w:
        padding = np.zeros((stack_height, right_w - left_w, 3), dtype=np.uint8)
        stacked_img_left = np.hstack((stacked_img_left, padding))
    elif left_w > right_w:
        padding = np.zeros((stack_height, left_w - right_w, 3), dtype=np.uint8)
        stacked_img_right = np.hstack((stacked_img_right, padding))


    final = np.vstack((stacked_img_left, stacked_img_right))
    cv2.imshow('masks', final)
