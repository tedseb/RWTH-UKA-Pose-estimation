import torch
import cv2

def plot_one_box(box, img, color=(128, 128, 128), label=None, line_thickness=3):
    # Plots one bounding box on image 'im' using OpenCV
    assert img.data.contiguous, 'Image not contiguous. Apply np.ascontiguousarray(im) to plot_on_box() input image.'
    line_thickness = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
    c_1, c_2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
    cv2.rectangle(img, c_1, c_2, color, thickness=line_thickness, lineType=cv2.LINE_AA)
    if label:
        font_thickness = max(line_thickness - 1, 1)  # font thickness
        font_scale = line_thickness / 3
        t_size = cv2.getTextSize(label, 0, fontScale=font_scale, thickness=font_thickness)[0]
        c_2 = c_1[0] + t_size[0], c_1[1] - t_size[1] - 3
        cv2.rectangle(img, c_1, c_2, color, -1, cv2.LINE_AA)  # filled
        org = (c_1[0], c_1[1] - 2)
        cv2.putText(img, label, org, 0, font_scale, [225, 255, 255], thickness=font_thickness, lineType=cv2.LINE_AA)

def get_index_box(index, pred):
    for det in pred:
        if len(det):
            for *box, conf, cls in det:
                if int(cls) == index:
                    box_list = [int(i) for i in box]
                    box_list.append(float(conf))
                    return box_list

    return None

class YoloWeightModel:
    def __init__(self, weigh_path = "./models/weights.pt"):
        self._model = torch.hub.load('ultralytics/yolov5', 'custom', path=weigh_path)
        self._color_codes = [(240,128,128), (30,144,255)]
        self._cls_labels = ["weight_left", "weight_right"]
        self._must_be_in_img = [1, 1]

    def inference(self, img):
        return self._model(img)

    def draw_pred_to_image(self, img, pred):
        image = img.copy()
        for det in pred:
            if len(det):
                for *xyxy, conf, cls in reversed(det):
                    cls_int = int(cls)
                    label = f"{self._cls_labels[cls_int]} {conf:.2f}"
                    plot_one_box(xyxy, image, label=label, color=self._color_codes[cls_int], line_thickness=3)
        return image

    def is_valid_prediction(self, pred):
        cls_len = len(self._cls_labels)
        cls_counter = [0] * cls_len
        for det in pred:
            if len(det):
                for *_, cls in det:
                    cls_counter[int(cls)] +=1

        for i, value in enumerate(self._must_be_in_img):
            if cls_counter[i] != value:
                return False
        return True

    def get_left_box(self, pred):
        index = self._cls_labels.index("weight_left")
        return get_index_box(index, pred)

    def get_right_box(self, pred):
        index = self._cls_labels.index("weight_right")
        return get_index_box(index, pred)
