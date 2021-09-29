#!/usr/bin/env python3
import numpy as np
import cv2
import tensorflow as tf
import time
import rospy
from sensor_msgs.msg import Image
from backend.msg import Person, Persons, Bodypart, Pixel,Bboxes
from std_msgs.msg import Float32MultiArray

import os
os.environ["CUDA_VISIBLE_DEVICES"] = "0"

from tensorflow.python.keras.backend import set_session

config = tf.compat.v1.ConfigProto()
config.gpu_options.allow_growth = True # dynamically grow the memory used on the GPU
config.log_device_placement = True # to log device placement (on which device the operation ran)
sess = tf.compat.v1.Session(config=config)
set_session(sess)

import matplotlib.pyplot as plt
plt.switch_backend('TkAgg')
# noinspection PyUnresolvedReferences
from mpl_toolkits.mplot3d import Axes3D

class run_metrabs():
    def __init__(self):
        #self.model = tf.saved_model.load('/home/trainerai/trainerai-core/src/AI/metrabs/models/metrabs_multiperson_smpl_combined')
        self.model = tf.saved_model.load('/home/trainerai/trainerai-core/src/AI/metrabs/models/metrabs_multiperson_smpl')
        image = tf.image.decode_jpeg(tf.io.read_file('/home/trainerai/trainerai-core/src/AI/metrabs/test_image_3dpw.jpg'))
        #self.intrinsics = tf.constant([[3324, 0, 1311], [0, 1803, 707], [0, 0, 1]], dtype=tf.float32)
        self.intrinsics = tf.constant([[1962, 0, 540], [0, 1969, 960], [0, 0, 1]], dtype=tf.float32)
        # Use your detector of choice to obtain bounding boxes.
        # See the README for how to combine the YOLOv4 detector with our MeTRAbs code.
        person_boxes = tf.constant([[0, 626, 367, 896], [524, 707, 475, 841], [588, 512, 54, 198]], tf.float32)

        poses3d = self.model.predict_single_image(image, self.intrinsics, person_boxes)
        #detections, poses3d, poses2d  = self.model.predict_single_image(image)
        poses2d = ((poses3d / poses3d[..., 2:]) @ tf.linalg.matrix_transpose(self.intrinsics))[..., :2]
        print("3D AI loaded")

        # define a publisher to publish the 3D skeleton of multiple people
        self.publisher = rospy.Publisher('personsJS', Persons, queue_size=2)
        self.publisher_crop = rospy.Publisher('cropImage', Image, queue_size=2)   


        # define a subscriber to retrive tracked bodies
        rospy.Subscriber('bboxes', Bboxes, self.callback_regress)
        #rospy.Subscriber('bboxes1', Bboxes, self.callback_regress)

        rospy.Subscriber('image', Image, self.callback_setImage)       
        #srospy.Subscriber('image1', Image, self.callback_setImage)
        self.spin()


    def spin(self):
        '''
        We enter in a loop and wait for exit whenever `Ctrl + C` is pressed
        '''
        rospy.spin()

    def callback_setImage(self, msg):
        self.msg_image = msg
        shape = self.msg_image.height, self.msg_image.width, 3     #(480, 640, 3) --> (y,x,3) = (h,w,3)
        img = np.frombuffer(self.msg_image.data, dtype=np.uint8)
        self.img_original_bgr = img.reshape(shape)
        self.img_original_bgr = cv2.cvtColor(self.img_original_bgr, cv2.COLOR_BGR2RGB)

    def callback_regress(self, body_bbox_list_station):
        '''
        This function will be called everytime whenever a message is received by the subscriber
        '''
        body_bbox_list_station_reshaped=np.array(body_bbox_list_station.data).reshape(-1,4)
        tmpTime = time.time()

        #ToDo: differ between someone that is focused on the station and someone that is going through the camera and let to occlusion. Currently take the skeleton that is the biggest
    
        # Body Pose Regression
        pred_output_list = self.model.predict_single_image(self.img_original_bgr, self.intrinsics, body_bbox_list_station_reshaped)
        poses2d = ((pred_output_list / pred_output_list[..., 2:]) @ tf.linalg.matrix_transpose(self.intrinsics))[..., :2]
        #self.visualize_pose((self.img_original_bgr), (pred_output_list.numpy()), (poses2d.numpy()), (self.model.crop_model.joint_edges.numpy()),(self.msg_image.header.stamp))
        
        fps = int(1/(time.time()-tmpTime))
        print("FPS for Metrabs : ",fps)
        self.publish_results(pred_output_list.numpy(), self.msg_image,  body_bbox_list_station.stationID,body_bbox_list_station.sensorID,body_bbox_list_station_reshaped )


    def publish_results(self,results, img_msg, stationID,sensorID,boxes):  
        if len(results) == 0:
            return
        inc=0
        msg = Persons()
        msg.header = img_msg.header
        msg.persons = list()

        cropImage=[]
        for idx,detection in enumerate(results):
            joints=detection
            bb=boxes[idx]
            croppedImg= self.img_original_bgr[int(bb[1]):int(bb[1]+bb[3]),int(bb[0]):int(bb[0]+bb[2])]
            #croppedImg= self.img_original_bgr[int(bb[1]):int(bb[1]+bb[3]),int(bb[0]-bb[3]/2+bb[2]/2):int(bb[0]+bb[2]/2+bb[3]/2)]
            cropImage.append(croppedImg)
        
            lenPoints=len(joints)       #ToDo use fix number of joints to save calculation time
            person_msg = Person()
            if len(stationID) >0:
                person_msg.stationID = int(stationID[inc])
                person_msg.sensorID = sensorID[inc]
            person_msg.bodyParts = [None]*lenPoints
            for idx in range(lenPoints):
                person_msg.bodyParts[idx] = Bodypart()
            for idx in range(lenPoints):
                person_msg.bodyParts[idx].score = 0.8
                person_msg.bodyParts[idx].pixel.x = joints[idx,0]
                person_msg.bodyParts[idx].pixel.y = joints[idx,1]
                person_msg.bodyParts[idx].point.x =joints[idx,0]/400-6
                person_msg.bodyParts[idx].point.y =joints[idx,2]/400-25
                person_msg.bodyParts[idx].point.z = -joints[idx,1]/400         #ToDo: Normalisierung ins Backend
            inc=inc+1
            msg.persons.append(person_msg)
        self.publisher.publish(msg)
        imgAdd=cv2.vconcat(cropImage)  
        msg_cropImage = Image()
        msg_cropImage.header.stamp = rospy.Time.now()
        msg_cropImage.header.frame_id = img_msg.header.frame_id
        msg_cropImage.encoding = "bgr8"
        msg_cropImage.data = np.array(imgAdd, dtype=np.uint8).tostring()
        msg_cropImage.height, msg_cropImage.width = imgAdd.shape[:-1]
        msg_cropImage.step = imgAdd.shape[-1]*imgAdd.shape[0]

        self.publisher_crop.publish(msg_cropImage)

    def visualize_pose(self,image, poses3d, poses2d, edges, x):
        # Matplotlib interprets the Z axis as vertical, but our poses have Y as the vertical axis.
        # Therefore we do a 90 degree rotation around the horizontal (X) axis
        poses3d[..., 1], poses3d[..., 2] = poses3d[..., 2], -poses3d[..., 1]

        fig = plt.figure(figsize=(10, 5.2))
        image_ax = fig.add_subplot(1, 2, 1)
        image_ax.set_title('Input')
        image_ax.imshow(image)

        for pose2d in poses2d:
            for i_start, i_end in edges:
                image_ax.plot(*zip(pose2d[i_start], pose2d[i_end]), marker='o', markersize=2)
            image_ax.scatter(pose2d[:, 0], pose2d[:, 1], s=2)

        pose_ax = fig.add_subplot(1, 2, 2, projection='3d')
        pose_ax.set_title('Prediction')
        range_ = 3000
        pose_ax.view_init(5, -85)
        pose_ax.set_xlim3d(-range_, range_)
        pose_ax.set_ylim3d(0, 2 * range_)
        pose_ax.set_zlim3d(-range_, range_)
        pose_ax.set_box_aspect((1, 1, 1))

        for pose3d in poses3d:
            for i_start, i_end in edges:
                pose_ax.plot(*zip(pose3d[i_start], pose3d[i_end]), marker='o', markersize=2)
            pose_ax.scatter(pose3d[:, 0], pose3d[:, 1], pose3d[:, 2], s=2)

        fig.tight_layout()
        #plt.show()
        plt.savefig('/home/trainerai/trainerai-core/src/AI/metrabs/rosSave/Ted_intrin' + str(x)+'.png')

if __name__ == '__main__':
    rospy.init_node('metrabs', anonymous=True)
    # instantiate the Comparator class
    run_spin_obj = run_metrabs()
