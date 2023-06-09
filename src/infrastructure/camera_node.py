#!/usr/bin/python3
from pathlib import Path
import time
import glob
import json
from threading import Lock
import rospy
import cv2
import logy
from sensor_msgs.msg import Image
from backend.msg import ImageData, PlayControl
import numpy as np
import argparse
import sys
from pytube import YouTube
from enum import Enum
from datetime import datetime

# from utils.imutils import crop_bboxInfo, process_image_bbox, process_image_keypoints, bbox_from_keypoints ToDo: Do cropping here.
VIDEO_DIR_PATH = "/home/trainerai/trainerai-core/data/videos/"
IMAGE_DIR_PATH = "/home/trainerai/trainerai-core/data/images"
PLAY_CONTROL_CHANNEL = "/image/play_control"

from std_msgs.msg import Int32

class VideoMode(Enum):
    INVALID = 0
    YOUTUBE = 1
    WEB_CAM = 2
    IP_CAM = 3
    DISK_VIDEO = 4

def returnCameraIndices():
    index = 0
    arr = []
    i = 10
    while i > 0:
        cap = cv2.VideoCapture(index)
        if cap.read()[0]:
            arr.append(index)
            cap.release()
        index += 1
        i -= 1
    return arr

class CameraNode():
    def __init__(self, verbose=False, dev_id=0, check_cameras=False, camera_mode=VideoMode.INVALID, video_info=None, debug_repetition_ms=0, channel="image", repetition=True):
        self._cap = None
        self._verbose = verbose
        self._camera_mode = camera_mode
        self._dev_id = "dev" + str(dev_id)
        self._youtube_mode = False
        self._disk_mode = False
        self._debug_repetition_ms = debug_repetition_ms
        self._paused = False
        self._repetition = repetition
        self._cur_frame = None
        self._cap_mutex = Lock()

        rospy.init_node('camera', anonymous=True)
        logy.debug("New Channel:", channel)
        logy.debug("Camera Mode:", camera_mode.name)
        logy.debug("Video Info:", video_info)
        self._pub = rospy.Publisher(channel, ImageData, queue_size=1)
        self._play_control_sub = rospy.Subscriber(PLAY_CONTROL_CHANNEL, PlayControl, self.callback_play_control)

        if self._camera_mode is VideoMode.INVALID:
            raise RuntimeError("Invalid video mode")
        elif self._camera_mode is VideoMode.YOUTUBE:
            url = f"https://www.youtube.com/watch?v={video_info}"
            self.set_youtube_stream(url)
            self._youtube_mode = True
        elif self._camera_mode is VideoMode.WEB_CAM:
            self.set_webcam(check_cameras, int(video_info))
        elif self._camera_mode is VideoMode.IP_CAM:
            self.set_ipcam(video_info)
        elif self._camera_mode is VideoMode.DISK_VIDEO:
            self.set_disk_video(video_info)
            self._disk_mode = True
        elif not self._cap.isOpened() or self._cap is None:
            self.set_youtube_stream()
            self._youtube_mode = True

    def __del__(self):
        if self._cap is not None:
            self._cap.release()

    def start_camera_publisher(self):
        rate = rospy.Rate(25)  # TODO: Aufnahme ist in 25FPS
        frame_num = 0

        logy.info("Camera node started")
        if self._debug_repetition_ms > 0:
            time_past = time.time() * 1000
            debug_id = 0

        while not rospy.is_shutdown():
            if not self._paused or self._cur_frame is None:
                with self._cap_mutex:
                    ret, self._cur_frame = self._cap.read()
            if not ret:
                if not self._repetition:
                    break
                if self._youtube_mode:
                    self._cap.open(self._disk_path)
                    continue
                elif self._disk_mode:
                    self._cap.open(self._disk_path)
                    continue
                break
                #rospy.logerr('Could not get image')
                #raise IOError('[CameraNode] Could not get image')

            #frame = cv2.resize(frame, (1280, 720))
            img = Image()
            img.header.stamp = rospy.Time.now()
            img.header.frame_id = self._dev_id
            img.encoding = "bgr8"
            img.data = np.array(self._cur_frame, dtype=np.uint8).tobytes()
            img.height, img.width = self._cur_frame.shape[:-1]
            img.step = self._cur_frame.shape[-1]*self._cur_frame.shape[0]
            msg = ImageData()
            msg.image = img
            msg.is_debug = False
            if self._debug_repetition_ms > 0:
                time_now = time.time() * 1000
                if time_now - time_past > self._debug_repetition_ms:
                    msg.is_debug = True
                    msg.debug_id = debug_id
                    logy.debug(f"#### Start Debug Frame {debug_id} ####", tag="debug_frame")
                    time_past = time_now
                    debug_id += 1
            msg.frame_num = frame_num
            self._pub.publish(msg)
            frame_num += 1
            rate.sleep()

    @logy.catch_ros
    def callback_play_control(self, play_control: PlayControl):
        action_type = int(play_control.action_type)
        action_payload = json.loads(str(play_control.action_payload))
        if action_type == 1:
            self._paused = action_payload["pause"]
        elif action_type == 2:
            with self._cap_mutex:
                offset = int(action_payload["offset"])
                current_frame = self._cap.get(cv2.CAP_PROP_POS_FRAMES)
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, max(current_frame + offset, 0))
        elif action_type == 3:
            if self._cur_frame is None:
                return
            screen_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            screen_str = f"screenshot_{screen_str}"
            Path(IMAGE_DIR_PATH).mkdir(parents=True, exist_ok=True)
            cv2.imwrite(f"{IMAGE_DIR_PATH}/{screen_str}.jpg", self._cur_frame)
        elif action_type == 4:
            if self._cur_frame is None:
                return
            percent = float(action_payload["percent"])
            with self._cap_mutex:
                total_frames = self._cap.get(cv2.CAP_PROP_FRAME_COUNT)
                logy.info(f"{total_frames} {percent} {int(total_frames * percent)}")
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, int(total_frames * percent))



    def start_dataset_recording_publisher(self, calculate_timestamps=True):
        """ Same as a camera publisher, only that it takes a video as input and publishes the time of the video in parallel. Stops after playback"""
        self._timecode_pub = rospy.Publisher('ma_validation_video_timing', Int32, queue_size=100)
        self.ma_validation_done_pub = rospy.Publisher('ma_validation_done', Int32, queue_size=100)
        # Get the original FPS of the video
        fps = self._cap.get(cv2.CAP_PROP_FPS)
        frame_count = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT))
        frame_no = 0
        # Our rate reflects the fps at which we WANT the video to play to match the normal rate of our AI
        publish_fps = 25
        rate = rospy.Rate(publish_fps)  # TODO: Aufnahme ist in 25FPS
        duration = frame_count/publish_fps
        while not rospy.is_shutdown() and self._cap.isOpened():
            if frame_count < frame_no:
                # For 10 more seconds, publish timecodes to trigger the ending of last exercise if it is not perfectly aligned with video.
                if calculate_timestamps:
                        timestamp = int(frame_no / fps)
                else:
                    timestamp = int(self._cap.get(cv2.CAP_PROP_POS_MSEC) / 1000)
                for i in range(10):
                    timestamp += 1
                    time_message = Int32()
                    time_message.data = timestamp
                    self._timecode_pub.publish(time_message)
                    rospy.sleep(1) # Sleep for 10 seconds before we trigger report creation and end the process
                self.ma_validation_done_pub.publish(Int32(0))
                return
            ret, frame = self._cap.read()
            if not ret:
                if self._disk_mode:
                    self._cap.open(self._disk_path)
                    continue

                rospy.logerr('Could not get image')
                raise IOError('[CameraNode] Could not get image')

            if calculate_timestamps:
                timestamp = int(frame_no / fps)
            else:
                timestamp = int(self._cap.get(cv2.CAP_PROP_POS_MSEC) / 1000)
            frame_no += 1
            # If there is a timestamp, we use it. Otherwise we use our own calculation
            time_message = Int32()
            time_message.data = timestamp
            self._timecode_pub.publish(time_message)
            if self._verbose:
                rospy.logerr_throttle(10, "\nData Set Recording is running...\nTime in original video: " + str(timestamp) + " seconds.\nFrame no. is " + str(frame_no) + "/ " + str(frame_count) + "\nVideo has " + str(fps) + "fps. \nPublishing at " + str(publish_fps) + " fps.\nEta: " + str(duration - (frame_no/publish_fps)) + " seconds.")

            frame = cv2.resize(frame, (1280,720))
            img = Image()
            img.header.stamp = rospy.Time.now()
            img.header.frame_id = self._dev_id
            img.encoding = "bgr8"
            img.data = np.array(frame, dtype=np.uint8).tobytes()
            img.height, img.width = frame.shape[:-1]
            img.step = frame.shape[-1]*frame.shape[0]
            msg = ImageData()
            msg.image = img
            msg.is_debug = False
            self._pub.publish(msg)
            rate.sleep()

    def set_youtube_stream(self, url = "https://youtu.be/bqpCkbAr8dY"):
        video_path = Path(VIDEO_DIR_PATH)
        if not video_path.is_dir():
            print("Create video dir")
            video_path.mkdir(parents=True, exist_ok=True)

        video_path = f"{VIDEO_DIR_PATH}{url[-11:]}*"
        video = None
        for file in glob.glob(video_path):
            #print("#########", file)
            video = file
            break

        if video is None:
            video = self.download_youtube_video(url, VIDEO_DIR_PATH)

        print("OPEN VIDEO: {video}")
        self._disk_path = video
        self._cap = cv2.VideoCapture(video)

    def download_youtube_video(self, url : str, download_path = VIDEO_DIR_PATH) -> str:
        yid = url[-11:]
        video = YouTube(url)
        data = video.streams.all()
        options = []
        # Get all possible youtube qualities
        for option in data:
            option = str(option)
            option = option[1:-1].split()
            option_dict = {}
            for setting in option[1:]:
                setting = setting.replace('"', '')
                setting = setting.split("=")
                option_dict[setting[0]] = setting[1]
            options.append(option_dict)

        # Select quality with highest resolution
        selected_option = None
        for option in options:
            if option["type"] != "video":
                continue

            option["res"] = int(option["res"][0:-1])
            if selected_option is None:
                selected_option = option
                continue

            if option["res"] > selected_option["res"]:
                selected_option = option

        itag = int(selected_option["itag"])
        ext = selected_option["mime_type"].split("/")[1]
        res = selected_option["res"]

        video = video.streams.get_by_itag(itag)
        filename = f"{yid}_{res}p.{ext}"
        video.download(output_path=download_path, filename=filename)
        return f"{download_path}{filename}"
        #print(f"tag={itag}, ext={ext}, res={res}p")

    def set_webcam(self, check_cameras = False, camera_index = 0):
        if check_cameras:
            indices = returnCameraIndices()
            if len(indices) == 0:
                rospy.logerr('No Cameras Found')
                raise IOError('[CameraNode] No Cameras Found')
            self._cap = cv2.VideoCapture(indices[0])
            if self._verbose:
                print(f"[CameraNode] start cam on index [indices[0]]")
        else:
            self._cap = cv2.VideoCapture(camera_index)

    def set_ipcam(self, camera_ip = None):
        self._cap = cv2.VideoCapture(f"rtsp://admin:Vergessen1@{camera_ip}")
        print(f"START IP: {camera_ip}")

    def set_disk_video(self, path = "/home/trainerai/trainerai-core/data/video.avi"):
        print("CAMERA PATH: ", path)
        self._disk_path = path
        self._cap = cv2.VideoCapture(path)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    parser.add_argument("-y", "--youtube-id", type=str, help="start on youtube ID")
    parser.add_argument("-c", "--check-cameras", help="Check all camera input id's from 0 to 10. Takes the first working match.", action="store_true")
    parser.add_argument("-i", "--camera-index", type=int, help="Open camera on opencv camera index")
    parser.add_argument("-p", "--ip", type=str, help="Start IP cam on ip")
    parser.add_argument("-r", "--repeat", default=True, type=bool, help="Repeat video")
    parser.add_argument("-k", "--disk", type=str, help="Start video from disk path. Relative to root")
    parser.add_argument("-d", "--dev-id", default=0, type=str, help="Ros msgs header transform dev{dev-id}")
    parser.add_argument("--channel", default="image", type=str, help="Image channel Name")
    parser.add_argument("--debug-frames", default=0, type=int, help="Debug Frame time in ms. At 0 there are no debug frames.")
    parser.add_argument("--dataset-recording", default=False, type=bool, help="True if this is a dataset recording session.")

    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]

    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()

    #Todo: In Ros Logger
    if args.verbose:
        rospy.loginfo("Verbosity turned on")
        if args.youtube_id is None:
            rospy.loginfo("Youtube turned on")
        if args.camera_index is None:
            rospy.loginfo(f"Try to open {args.camera_index}")
        if args.check_cameras:
            rospy.loginfo("Try to find camera index")
        if args.ip is None:
            rospy.loginfo(f"Try to find ip camera on {args.ip}")
        rospy.loginfo(f"Send on dev{args.dev_id}")

    mode = VideoMode.INVALID

    if args.youtube_id is not None:
        mode = VideoMode.YOUTUBE
        info = args.youtube_id
    elif args.camera_index is not None or args.check_cameras:
        mode = VideoMode.WEB_CAM
        info = args.camera_index
    elif args.ip is not None:
        mode = VideoMode.IP_CAM
        info = args.ip
    elif args.disk is not None:
        mode = VideoMode.DISK_VIDEO
        info = args.disk

    logy.basic_config(debug_level=logy.DEBUG, module_name="CAMERA")

    repetition = args.repeat
    try:
        print("INFO:", info)
        node = CameraNode(args.verbose, args.dev_id, args.check_cameras, mode, info, debug_repetition_ms=args.debug_frames, channel=args.channel, repetition=repetition)
        if args.dataset_recording:
            node.start_dataset_recording_publisher()
        else:
            node.start_camera_publisher()
    except rospy.ROSInterruptException:
        pass

