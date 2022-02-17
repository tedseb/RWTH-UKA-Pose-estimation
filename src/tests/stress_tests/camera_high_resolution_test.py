#!/usr/bin/python3
from pathlib import Path
import time
import glob
import subprocess
import rospy
import psutil
import cv2
import logy
from sensor_msgs.msg import Image
from backend.msg import ImageData
import numpy as np
import argparse
import sys
from pytube import YouTube
from std_msgs.msg import String

# from utils.imutils import crop_bboxInfo, process_image_bbox, process_image_keypoints, bbox_from_keypoints ToDo: Do cropping here.
VIDEO_DIR_PATH = "/home/trainerai/trainerai-core/data/videos/"

class Resolution:
    PX720 = (1280, 720)
    PX1080 = (1280, 1080)
    PX1440 = (2560, 1440)
    PX2160 = (3840, 2160)

class CameraNode():

    def __init__(self, url_id="YlCsrVjr3hk", step_time_s=15, steps=None):
        self._cap = None
        self._step_time = step_time_s
        self._url_id = url_id
        self._steps = [(25, Resolution.PX720), (25, Resolution.PX1080), (25, Resolution.PX1440), (25, Resolution.PX2160)]
        self._cur_step = 0
        self._last_step_time = 0.0
        self._dev_id = "999"
        self._debug_repetition_ms = 0
        if steps is not None:
            self._steps = steps

        rospy.set_param('weight_frames', {"empty" : 0})
        static_transform_path = "/home/trainerai/trainerai-core/src/station_manager/launch/static_transform.launch"
        self.__transform_process = subprocess.Popen(["roslaunch", static_transform_path, "dev:=999"])
        self._pub = rospy.Publisher('image', ImageData, queue_size=1)
        self._publisher_pull_param = rospy.Publisher('pull_param', String , queue_size=2)
        self._last_step_start = 0


        url = f"https://www.youtube.com/watch?v={url_id}"
        self.set_youtube_stream(url)


        if self._cap is None or not self._cap.isOpened():
            logy.error(f"Could not open video {url_id}")

        step_data = self._steps[self._cur_step]
        logy.info(f"--- New Step ({self._cur_step}), resolution={str(step_data[1])}, fps = {step_data[0]} ---")
        self._station_frame_parameters = {"999" : {}}
        self._cur_step = 0
        self._last_step_time = time.time()
        self.set_step(self._steps[self._cur_step][1])

    def __del__(self):
        self.__transform_process.terminate()
        try:
            self.__transform_process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.kill(self.__transform_process.pid)

    def kill(self, proc_pid):
        process = psutil.Process(proc_pid)
        for proc in process.children(recursive=True):
            proc.kill()
        process.kill()

    def set_step(self, res):
        frame = [0, 0, res[0], res[1]]
        self._station_frame_parameters["999"][self._dev_id] = frame
        rospy.set_param('station_frames', self._station_frame_parameters)
        self._publisher_pull_param.publish("True")

    def deactivate_station(self):
        rospy.set_param('station_frames', {})
        self._publisher_pull_param.publish("True")

    def check_step(self) -> bool:
        if time.time() - self._last_step_time >= self._step_time:
            if self._cur_step + 1 >= len(self._steps):
                return False

            self._cur_step += 1
            self._last_step_time = time.time()
            step_data = self._steps[self._cur_step]
            self.set_step(step_data[1])
            logy.info(f"--- New Step ({self._cur_step}), resolution={str(step_data[1])}, fps = {step_data[0]} ---")
        return True

    def start_camera_publisher(self):
        logy.info("Camera node started")
        #time_test = time.time()

        if self._debug_repetition_ms > 0:
            time_past = time.time() * 1000
            debug_id = 0

        while not rospy.is_shutdown():
            time_temp = time.time()
            ret, frame = self._cap.read()
            if not ret:
                self._cap.open(self._disk_path)
                continue

            if not self.check_step():
                break

            frame = cv2.resize(frame, self._steps[self._cur_step][1])
            img = Image()
            img.header.stamp = rospy.Time.now()
            img.header.frame_id = "dev" + self._dev_id
            img.encoding = "bgr8"
            img.data = np.array(frame, dtype=np.uint8).tobytes()
            img.height, img.width = frame.shape[:-1]
            img.step = frame.shape[-1]*frame.shape[0]
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
            self._pub.publish(msg)

            #fps = time.time() - time_test
            #time_test = time.time()
            #print("fps local=", 1 / fps)
            logy.log_fps("camera_fps")

            time_temp = time.time() - time_temp
            #print("time_temp:", time_temp * 1000)
            time_sleep = max(0, (1 / self._steps[self._cur_step][0]) - time_temp)
            #print("time_sleep:", time_sleep * 1000)
            time.sleep(time_sleep)


    def set_youtube_stream(self, url="https://youtu.be/bqpCkbAr8dY"):
        video_path = Path(VIDEO_DIR_PATH)
        if not video_path.is_dir():
            print("Create video dir")
            video_path.mkdir(parents=True, exist_ok=True)

        video_path = f"{VIDEO_DIR_PATH}{url[-11:]}*"
        video = None
        for file in glob.glob(video_path):
            video = file
            break

        if video is None:
            video = self.download_youtube_video(url, VIDEO_DIR_PATH)

        logy.debug(f"open video {video}")
        self._disk_path = video
        self._cap = cv2.VideoCapture(video)

    def download_youtube_video(self, url : str, download_path=VIDEO_DIR_PATH) -> str:
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

if __name__ == '__main__':
    print(sys.argv)
    logy.basic_config(logy.DEBUG, "CAM_STRESS_TEST")
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--step-time", default=15, type=int, help="Time in s before next step")
    args = parser.parse_args()

    logy.basic_config(debug_level=logy.DEBUG, module_name="CAMERA_TEST")
    rospy.init_node('camera_stress_test', anonymous=True)

    try:
        node = CameraNode(step_time_s=args.step_time)
        node.start_camera_publisher()
    except rospy.ROSInterruptException:
        pass
