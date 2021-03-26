import numpy as np
import cv2
import pafy
import matplotlib.pyplot as plt
from matplotlib import cm
from PIL import Image as ImagePil
from sensor_msgs.msg import Image
import time
import rospy

import torch
from torch import nn
from torchvision import transforms

utils = torch.hub.load('NVIDIA/DeepLearningExamples:torchhub', 'nvidia_ssd_processing_utils')
url = "https://www.youtube.com/watch?v=wqctLW0Hb_0"



def run_objectdetector(img_msg):
    shape = img_msg.height, img_msg.width, 3                            #(480, 640, 3) --> (y,x,3)
    img = np.frombuffer(img_msg.data, dtype=np.uint8)
    img_original_bgr = img.reshape(shape)
  
    tmpTime = time.time()
    
    body_bbox=[]

    img = obj_detect(img_original_bgr)[:,:,::-1]
   
    fps = int(1/(time.time()-tmpTime))
    print("FPS : ",fps)

    msg_cropImage = Image()
    msg_cropImage.header.stamp = rospy.Time.now()
    msg_cropImage.header.frame_id = 'dev0'
    msg_cropImage.encoding = "bgr8"
    msg_cropImage.data = np.array(img, dtype=np.uint8).tostring()
    msg_cropImage.height, msg_cropImage.width = img.shape[:-1]
    msg_cropImage.step = img.shape[-1]*img.shape[0]

    publisher_crop.publish(msg_cropImage)

class ObjectDetectionPipeline:
    def __init__(self, threshold=0.4, device="cpu", cmap_name="tab10_r"):
        # First we need a Transform object to turn numpy arrays to normalised tensors.
        # We are using an SSD300 model that requires 300x300 images.
        # The normalisation values are standard for pretrained pytorch models.
        self.tfms = transforms.Compose([
            transforms.Resize(300),
            transforms.CenterCrop(300),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225])
        ])

        # Next we need a model. We're setting it to evaluation mode and sending it to the correct device.
        # We get some speedup from the gpu but not as much as we could.
        # A more efficient way to do this would be to collect frames to a buffer,
        # run them through the network as a batch, then output them one by one
        precision = 'fp32'
        self.model = torch.hub.load('NVIDIA/DeepLearningExamples:torchhub', 'nvidia_ssd', model_math=precision).eval().to(device)
   

        # Stop the network from keeping gradients.
        # It's not required but it gives some speedup / reduces memory use.
        for param in self.model.parameters():
            param.requires_grad = False


        self.device = device
        self.threshold = threshold # Confidence threshold for displaying boxes.
        self.cmap = cm.get_cmap(cmap_name) # colour map
        self.classes_to_labels = utils.get_coco_object_dictionary()


    @staticmethod
    def _crop_img(img):
        """Crop an image or batch of images to square. We can also leave it as reactangle"""
        if len(img.shape) == 3:
            y = img.shape[0]
            x = img.shape[1]
        elif len(img.shape) == 4:
            y = img.shape[1]
            x = img.shape[2]
        else:
            raise ValueError(f"Image shape: {img.shape} invalid")

        out_size = min((y, x))
        startx = x // 2 - out_size // 2
        starty = y // 2 - out_size // 2

        if len(img.shape) == 3:
            return img[starty:starty+out_size, startx:startx+out_size]
        elif len(img.shape) == 4:
            return img[:, starty:starty+out_size, startx:startx+out_size]

    def _plot_boxes(self, output_img, labels, boxes):
        """Plot boxes on an image"""
        for label, (x1, y1, x2, y2) in zip(labels, boxes):
            if (x2 - x1) * (y2 - y1) < 0.25:
                # The model seems to output some large boxes that we know cannot be possible.
                # This is a simple rule to remove them.

                x1 = int(x1*output_img.shape[1])
                y1 = int(y1*output_img.shape[0])
                x2 = int(x2*output_img.shape[1])
                y2 = int(y2*output_img.shape[0])
                print("x,y", x1 ,x2,y1,y2)
                rgba = self.cmap(label)
                bgr = rgba[2]*255, rgba[1]*255, rgba[0]*255
                cv2.rectangle(output_img, (x1, y1), (x2, y2), bgr, 2)
                cv2.putText(output_img, self.classes_to_labels[label - 1], (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)

        return output_img

    def __call__(self, img):
        """
        Now the call method This takes a raw frame from opencv finds the boxes and draws on it.
        """
        if type(img) == np.ndarray:
            # single image case

            # First convert the image to a tensor, reverse the channels, unsqueeze and send to the right device.
            img_tens = self.tfms(ImagePil.fromarray(img[:,:,::-1])).unsqueeze(0).to(self.device)

            # Run the tensor through the network.
            # We'll use NVIDIAs utils to decode.
            results = utils.decode_results(self.model(img_tens))
            boxes, labels, conf = utils.pick_best(results[0], self.threshold)

            # Crop the image to match what we've been predicting on.
            output_img = self._crop_img(img)
            print("Boxes", boxes)
            return self._plot_boxes(output_img, labels, boxes)

        elif type(img) == list:
            # batch case
            if len(img) == 0:
                # Catch empty batch case
                return None

            tens_batch = torch.cat([self.tfms(ImagePil.fromarray(x[:,:,::-1])).unsqueeze(0) for x in img]).to(self.device)
            results = utils.decode_results(self.model(tens_batch))

            output_imgs = []
            for im, result in zip(img, results):
                boxes, labels, conf = utils.pick_best(result, self.threshold)
                output_imgs.append(self._plot_boxes(self._crop_img(im), labels, boxes))

            return output_imgs

        else:
            raise TypeError(f"Type {type(img)} not understood")
def myhook():
    rospy.loginfo("Shutdown")
    rospy.signal_shutdown("Darum")

global obj_detect
obj_detect = ObjectDetectionPipeline(device="cuda", threshold=0.5)

rospy.init_node('objectNodeSSD', anonymous=True)
rospy.Subscriber('image', Image, run_objectdetector)
publisher_crop = rospy.Publisher('imageSSD', Image , queue_size=2)
rospy.on_shutdown(myhook)
rospy.spin()