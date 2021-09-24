import cv2
from src.utils import draw_boxes, get_contours, get_masks, crop_weights_into_image, crop_masks_into_image
from src.contour_boxes import ContourBoxes
from src.weight_boxes import WeightBoxes
#pylint: disable=unused-wildcard-import
from src.config import *

class Dumbbell:
    def __init__(self, left_box, right_box, img, color_ranges):
        #self._merged_boxes = []
        self._color_ranges = color_ranges
        self._left_box = left_box
        self._right_box = right_box
        self._box_contours_left = ContourBoxes()
        self._box_contours_right = ContourBoxes()
        self._masks_left = []
        self._masks_right = []
        self._colors = [(0, 255, 255), (0, 0, 255), (255, 0, 0), (0, 255, 0)]
        left_img = img[left_box[1]:left_box[3], left_box[0]:left_box[2]]
        right_img = img[right_box[1]:right_box[3], right_box[0]:right_box[2]]

        self.add_masks_left(left_img, False)
        self.add_masks_right(right_img, False)

    def add_masks(self, img, combine_bitwise = True):
        left_img = img[self._left_box[1]:self._left_box[3], self._left_box[0]:self._left_box[2]]
        right_img = img[self._right_box[1]:self._right_box[3], self._right_box[0]:self._right_box[2]]
        self.add_masks_left(left_img, combine_bitwise)
        self.add_masks_right(right_img, combine_bitwise)

    def add_masks_left(self, img, combine_bitwise = True):
        masks = get_masks(img, self._color_ranges)
        cnts = get_contours(masks)
        self._box_contours_left.add_contours(cnts)
        if combine_bitwise:
            self._masks_left[0] = masks[0]
            self._masks_left[1:] = self._combine_masks(self._masks_left[1:], masks[1:])
        else:
            self._masks_left = masks

    def add_masks_right(self, img, combine_bitwise = True):
        masks = get_masks(img, self._color_ranges)
        cnts = get_contours(masks)
        self._box_contours_right.add_contours(cnts)
        if combine_bitwise:
            self._masks_right[0] = masks[0]
            self._masks_right[1:] = self._combine_masks(self._masks_right[1:], masks[1:])
        else:
            self._masks_right = masks

    def _combine_masks(self, masks_input, new_masks):
        if len(masks_input) != len(new_masks):
            raise RuntimeError("Masks must have same length")

        for i, new_mask in enumerate(new_masks):
            masks_input[i] = cv2.bitwise_or(masks_input[i], new_mask)
        return masks_input

    def barcodes(self):
        boxes_left = self._box_contours_left.merge_boxes_vertical()
        boxes_right = self._box_contours_right.merge_boxes_vertical()

        weight_boxes_left = WeightBoxes(boxes_left.copy(), self._masks_left.copy())
        weight_boxes_right = WeightBoxes(boxes_right.copy(), self._masks_right.copy())
        weight_boxes_left.get_neighbour()
        weight_boxes_right.get_neighbour()
        barcode_left = weight_boxes_left.check_neighbour()
        barcode_right = weight_boxes_right.check_neighbour()
        return (barcode_left, barcode_right)

    def draw_boxes(self, img):
        boxes_left = self._box_contours_left.merge_boxes_vertical()
        boxes_right = self._box_contours_right.merge_boxes_vertical()

        left_img = img[self._left_box[1]:self._left_box[3], self._left_box[0]:self._left_box[2]]
        right_img = img[self._right_box[1]:self._right_box[3], self._right_box[0]:self._right_box[2]]
        draw_boxes(boxes_left, (255, 0, 0), left_img, 1)
        draw_boxes(boxes_right, (255, 0, 0), right_img, 1)
        return crop_weights_into_image(img, left_img, right_img)

    def crop_weights_into_image(self, image):
        boxes_left = self._box_contours_left.merge_boxes_vertical()
        boxes_right = self._box_contours_right.merge_boxes_vertical()

        weight_boxes_left = WeightBoxes(boxes_left.copy(), self._masks_left.copy())
        weight_boxes_right = WeightBoxes(boxes_right.copy(), self._masks_right.copy())
        left_img = image[self._left_box[1]:self._left_box[3], self._left_box[0]:self._left_box[2]]
        right_img = image[self._right_box[1]:self._right_box[3], self._right_box[0]:self._right_box[2]]
        boxes_left1, boxes_left2 = weight_boxes_left.get_neighbour()
        boxes_right1, boxes_right2 = weight_boxes_right.get_neighbour()
        draw_boxes(boxes_left1, (0, 255, 0), left_img, 1)
        draw_boxes(boxes_left2, (0, 0, 255), left_img, 1)
        draw_boxes(boxes_right1, (0, 255, 0), right_img, 1)
        draw_boxes(boxes_right2, (0, 0, 255), right_img, 1)
        return crop_weights_into_image(image, left_img, right_img)

    def crop_masks_into_image_and_display(self, img):
        left_img = img[self._left_box[1]:self._left_box[3], self._left_box[0]:self._left_box[2]]
        right_img = img[self._right_box[1]:self._right_box[3], self._right_box[0]:self._right_box[2]]
        crop_masks_into_image(left_img, right_img, self._masks_left, self._masks_right, self._colors)
