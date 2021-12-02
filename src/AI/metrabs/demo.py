#!/usr/bin/env python3
import os
import numpy as np
import cv2
#import cv2
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
import tensorflow as tf
import time
from tensorflow.python.keras.backend import set_session

config = tf.compat.v1.ConfigProto()
config.gpu_options.allow_growth = True # dynamically grow the memory used on the GPU
config.log_device_placement = True # to log device placement (on which device the operation ran)
sess = tf.compat.v1.Session(config=config)
set_session(sess)

AI_HEIGHT = 720
AI_WIDTH = 1280

def main():
    #demo_with_just_an_image()
    #demo_with_known_intrinsics_and_boxes()
    #image = tf.image.decode_jpeg(tf.io.read_file('./image.jpg'))
    image = np.empty([AI_HEIGHT, AI_WIDTH, 3], dtype=np.uint8)
    model = tf.saved_model.load('./models/metrabs_multiperson_smpl')
    test_single_image(model, image)
    test_multi_image(model, image)

def test_single_image(model, image):
    #image = tf.image.decode_jpeg(tf.io.read_file('img/test_image_3dpw.jpg'))
    skeleton = 'smpl_24'
    person_boxes = np.array([670, 170, 200, 510], dtype=np.float32)
    intrinsics =  np.array([[1962, 0, 540], [0, 1969, 960], [0, 0, 1]], np.float32)
    person_boxes = [person_boxes]
    print()
    intrinsics =  np.array([[1962, 0, 540], [0, 1969, 960], [0, 0, 1]], np.float32)

    for i in range(2):
        print(f"Single Detection Boxes {i + 1}:")
        time_stamp = time.time()
        pred = model.predict_single_image(image, intrinsics, person_boxes)
        time_stamp = (time.time() - time_stamp) * 1000
        print(f"- TIME: {time_stamp} ms")


def test_multi_image(model, image):
    _test_multi_image(model, image, 1)
    _test_multi_image(model, image, 2)
    _test_multi_image(model, image, 4)
    _test_multi_image(model, image, 8)
    _test_multi_image(model, image, 16)

def _test_multi_image(model, image, count, repetitions = 2):
    print()
    intrinsics =  tf.constant([[[1962, 0, 540], [0, 1969, 960], [0, 0, 1]]], dtype=tf.float32)
    #person_boxes = np.array([670, 170, 200, 510], dtype=np.float32)
    person_boxes = np.array([670, 170, 200, 510], np.float32)
    images, ragged_boxes = multiply_images_and_boxes(count, image.copy(), [person_boxes.copy()])

    for i in range(repetitions):
        print(f"Multi Detection ({images.shape}) {i + 1}:")
        time_stamp = time.time()
        pred = model.predict_multi_image(images, intrinsics, ragged_boxes)
        time_stamp = (time.time() - time_stamp) * 1000
        print(f"- TIME: {time_stamp} ms")

def multiply_images_and_boxes(array_len, image, person_boxes):
    images = []
    boxes = []
    for _ in range(array_len):
        images.append(image)
        boxes.append(person_boxes)
    images = np.stack(images)
    ragged_boxes = tf.ragged.constant(boxes, ragged_rank=1)
    #ragged_boxes = tf.RaggedTensor.from_tensor(boxes)
    return images, ragged_boxes

def demo_with_just_an_image():
    model = tf.saved_model.load('./models/metrabs_multiperson_smpl_combined')
    for x in range(1000):
        x=x+6000
        print(x)
        image = tf.image.decode_jpeg(tf.io.read_file('./TedAllein&Traurig/Ted'+ str(x) +'.jpg'))
        detections, poses3d, poses2d = model.predict_single_image(image)
        #visualize_pose(image.numpy(), poses3d.numpy(), poses2d.numpy(), model.joint_edges.numpy(),x)
    print("fertig")


def demo_with_known_intrinsics_and_boxes():
    model = tf.saved_model.load('./models/metrabs_multiperson_smpl')
    image = tf.image.decode_jpeg(tf.io.read_file('./image.jpg'))

    intrinsics =  np.array([[1962, 0, 540], [0, 1969, 960], [0, 0, 1]], np.float32)
    person_boxes =  np.array([[670, 170, 200, 510]], np.float32)
    #intrinsics = tf.constant([[1962, 0, 540], [0, 1969, 960], [0, 0, 1]], dtype=tf.float32)
    #person_boxes = tf.constant([[670, 170, 200, 510]], tf.float32)


    for x in range(1000):
        x=x+6000
        print(x)
        image = tf.image.decode_jpeg(tf.io.read_file('./TedAllein&Traurig/Ted'+ str(x) +'.jpg'))
        poses3d = model.predict_single_image(image,intrinsics, person_boxes)
        poses2d = ((poses3d / poses3d[..., 2:]) @ tf.linalg.matrix_transpose(intrinsics))[..., :2]
        visualize_pose(image.numpy(), poses3d.numpy(), poses2d.numpy(), model.crop_model.joint_edges.numpy(),x)
    print("fertig")





def visualize_pose(image, poses3d, poses2d, edges, x):
    import matplotlib.pyplot as plt
    plt.switch_backend('TkAgg')
    # noinspection PyUnresolvedReferences
    from mpl_toolkits.mplot3d import Axes3D

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
    range_ = 1500
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
    plt.savefig('./TedAllein&TraurigAfter/Ted_intrin' + str(x)+'.png')


if __name__ == '__main__':
    main()
