#!/usr/bin/python3
from typing import Set
import pathlib
import subprocess
import argparse
import sys
import psutil
from src import DataManager, ParamUpdater, VideoSelection, StationSelection
import rospy
from rospy.exceptions import ROSException
from backend.msg import StationUsage
from backend.srv import WeightDetection, WeightDetectionResponse, WeightDetectionRequest
from src.config import *

class ComputerWorkload:
    def __init__(self):
        self.pc_id = 0
        self.gpu_id = 0
        self.cameras = {}
        self.stations = set({})

class StationManager():
    def __init__(self, debug_mode=False, verbose=False):
        rospy.init_node('param_updater', anonymous=True)
        rospy.Subscriber('station_usage', StationUsage, self.station_usage_callback)
        try:
            rospy.wait_for_service('ai/weight_detection', 10)
        except ROSException:
            LOG_ERROR("Time out on channel  'ai/weight_detection'")
        self._ai_weight_detection = rospy.ServiceProxy('ai/weight_detection', WeightDetection)

        self._weigth_detection = rospy.Service('sm/weight_detection', WeightDetection, self.handle_weight_detection_request)
        self._data_manager = DataManager()
        #Todo: verbose in args
        self._verbose = verbose or VERBOSE_MODE
        self._param_updater = ParamUpdater(self._data_manager,  self._verbose)
        self._active_cameras : Set[int] = set({})
        self._active_stations : Set[int] = set({})
        self._path_camera_node = str(pathlib.Path(__file__).parent.parent.absolute()) + "/CameraNode.py"
        self._path_transform_node = str(pathlib.Path(__file__).parent.absolute()) + "/launch/static_transform.launch"
        self._camera_process = {}
        self._transform_process = {}
        self._workload = []
        self._debug_mode = debug_mode or DEBUG_MODE

        if self._debug_mode:
            self.video_selection_gui = VideoSelection(self._data_manager)
            self.video_selection_gui.set_callback(station_manager.debug_callback)
            self.video_selection_gui.show()


        self.station_selection_gui = StationSelection(self._data_manager)
        self.station_selection_gui.show()

    def __del__(self):
        for cam_index in self._active_cameras:
            self.stop_camera(cam_index)

    def start(self):
        LOG_DEBUG("Started StationManager", self._verbose)
        rospy.spin()

    def station_usage_callback(self, msg):
        LOG_DEBUG("Station Usage Callback", self._verbose)

        self._param_updater.set_station(msg)
        cameras = self._param_updater.get_active_cameras()

        if self._debug_mode:
            return

        turn_on = cameras - self._active_cameras
        turn_off = self._active_cameras - cameras

        for cam_index in turn_on:
            self.start_camera(cam_index)

        for cam_index in turn_off:
            self.stop_camera(cam_index)

        LOG_DEBUG("Call weight detection service", self._verbose)
        result :  WeightDetectionResponse = self._ai_weight_detection("Test")
        LOG_DEBUG(f"Weight detection result = {result.weight}, code = {result.response}", self._verbose)

    def start_camera(self, camera_id : int):
        LOG_DEBUG(f"Start Camera with id {camera_id}", self._verbose)
        #DEVNULL = open(os.devnull, 'wb')
        cam_type = self._data_manager.get_camera_type(camera_id)
        cam_info = self._data_manager.get_camera_type_info(camera_id)

        LOG_DEBUG(f"start cam type {cam_type}, on {cam_info}", self._verbose)

        if cam_type == 0:
            args = f"-y {cam_info} -d {camera_id}"
        if cam_type == 1:
            args = f"-i {cam_info} -d {camera_id}"
        if cam_type == 2:
            args = f"-p {cam_info} -d {camera_id}"

        if self._verbose:
            args += " -v"

        self._camera_process[camera_id] = subprocess.Popen([self._path_camera_node] + args.split())
        self._transform_process[camera_id] = subprocess.Popen(["roslaunch", self._path_transform_node, f"dev:={camera_id}"])
        self._active_cameras.add(camera_id)

    def start_webcam(self, camera_id : int):
        print(f"Start webcam on index: {camera_id}")

    def start_ipcam(self, camera_id : int):
        print(f"Start webcam on index: {camera_id}")

    def stop_camera(self, camera_id : int):
        LOG_DEBUG(f"Stop Camera {camera_id}", self._verbose)
        if camera_id in self._camera_process:
            self._camera_process[camera_id].terminate()
            try:
                self._camera_process[camera_id].wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.kill(self._camera_process[camera_id].pid)
            del self._camera_process[camera_id]

        if camera_id in self._transform_process:
            self._transform_process[camera_id].terminate()
            try:
                self._transform_process[camera_id].wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.kill(self._transform_process[camera_id].pid)
            del self._transform_process[camera_id]

    def debug_callback(self, index : int):
        if not self._verbose:
            return

        print("Active Cameras: ", self._active_cameras)
        for cam_index in self._active_cameras:
            self.stop_camera(cam_index)

        self.start_camera(index)

    def kill(self, proc_pid):
        process = psutil.Process(proc_pid)
        for proc in process.children(recursive=True):
            proc.kill()
        process.kill()

    def handle_weight_detection_request(self, req : WeightDetectionRequest):
        print("test")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--debug", help="Only one camera with QT selection", action="store_true")
    parser.add_argument("-v", "--verbose", help="Verbose mode", action="store_true")
    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]
    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()

    station_manager = StationManager(debug_mode=args.debug, verbose=args.verbose)
    station_manager.start()
