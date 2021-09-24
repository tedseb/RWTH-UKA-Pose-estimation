import numpy as np
import cv2
import copy

class WeightBoxes:
    def __init__(self, boxes, masks):
        self._boxes = boxes.copy()
        self._left_neightboors = []
        self._right_neightboors = []
        self._masks = masks.copy()
        self.image_width = np.array(masks[0]).shape[1]

    def get_neighbour(self):
        for box in self._boxes:
            h_quarter = int(box[3] / 4)
            w_half = int(box[2] / 2)
            y = box[1] + h_quarter

            x_left = box[0] - w_half
            x_left = max(0, x_left)
            self._left_neightboors.append([x_left, y, box[2], h_quarter * 2])

            x_right = box[0] + w_half
            x_right = min(x_right, self.image_width - w_half)
            self._right_neightboors.append([x_right, y, box[2], h_quarter * 2])
        return copy.deepcopy(self._left_neightboors), copy.deepcopy(self._right_neightboors)

    def check_neighbour(self):
        color_codes = []
        for i, box_l in enumerate(self._left_neightboors):
            box_r = self._right_neightboors[i]
            tmp_sizes= []

            for mask in self._masks[1:]:
                cropped_left = mask[box_l[1]:box_l[1] + box_l[3], box_l[0]:box_l[0] + box_l[2]]
                cropped_right = mask[box_r[1]:box_r[1] + box_r[3], box_r[0]:box_r[0] + box_r[2]]
                contours_left = cv2.findContours(cropped_left.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                contours_right = cv2.findContours(cropped_right.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

                if (len(contours_left) > 0):
                    cnt = max(contours_left, key=cv2.contourArea)
                    (_, _), (w, h), _ = cv2.minAreaRect(cnt)
                    tmp_sizes.append(int(w * h))
                else:
                    tmp_sizes.append(0)
                
                if (len(contours_right) > 0):
                    cnt = max(contours_right, key=cv2.contourArea)
                    (_, _), (w, h), _ = cv2.minAreaRect(cnt)
                    tmp_sizes.append(int(w * h))
                else:
                    tmp_sizes.append(0)

            color_codes.append(int(np.array(tmp_sizes).argmax() / 2) + 1)
        return color_codes
