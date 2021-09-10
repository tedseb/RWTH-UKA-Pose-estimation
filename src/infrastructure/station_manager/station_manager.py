#!/usr/bin/python3
from typing import Set, Dict
from multiprocessing import Lock
import pathlib
import subprocess
import argparse
import sys
import signal
import psutil

from src import DataManager, CameraStationController, VideoSelection, StationSelection
from src.server import ServerController, ServerSocket, ResponseAnswer
import rospy
from rospy.exceptions import ROSException
from backend.msg import StationUsage, WeightColor
from backend.srv import WeightDetection, WeightDetectionResponse, WeightDetectionRequest
from src.config import *
from twisted.internet import reactor

def signal_handler(signal, frame):
    print("EXIT")
    reactor.callFromThread(reactor.stop)

class ComputerWorkload:
    def __init__(self):
        self.pc_id = 0
        self.gpu_id = 0
        self.cameras = {}
        self.stations = set({})

class StationManager():
    def __init__(self, debug_mode=False, verbose=False):
        rospy.init_node('param_updater', anonymous=True)
        #rospy.Subscriber('station_usage', StationUsage, self.station_usage_callback)
        # try:
        #     rospy.wait_for_service('ai/weight_detection', 0.5)
        # except ROSException:
        #     LOG_ERROR("Time out on channel  'ai/weight_detection'")
        # self._ai_weight_detection = rospy.ServiceProxy('ai/weight_detection', WeightDetection)

        self._data_manager = DataManager()
        #Todo: verbose in args
        self._verbose = verbose or VERBOSE_MODE
        self._debug_mode = debug_mode or DEBUG_MODE

        self._path_camera_node = str(pathlib.Path(__file__).absolute().parent.parent) + "/CameraNode.py"
        self._path_transform_node = str(pathlib.Path(__file__).absolute().parent) + "/launch/static_transform.launch"
        self._path_station_selection = str(pathlib.Path(__file__).absolute().parent) + "/src/station_selection.py"
        print(__file__)
        #self._path_camera_node = str(pathlib.Path(__file__).parent.parent.parent.parent.absolute()) + "/CameraNode.py"
        #self._path_transform_node = str(pathlib.Path(__file__).parent.parent.absolute()) + "/launch/static_transform.launch"
        print(self._path_camera_node)
        print(self._path_transform_node)

        self._station_selection_process = subprocess.Popen([self._path_station_selection])

        if self._debug_mode:
            self.video_selection_gui = VideoSelection(self._data_manager)
            self.video_selection_gui.set_callback(station_manager.debug_callback)
            self.video_selection_gui.show()

        # Thread Shared Data. Don't Use without Mutex lock!!!
        self._active_stations : Dict[int, str] = {}
        self._camera_process = {}
        self._transform_process = {}
        self._param_updater = CameraStationController(self._data_manager,  self._verbose)

        #Mutex 
        self._active_stations_mutex = Lock()
        self._camera_process_mutex = Lock()
        self._param_updater_mutex = Lock()

        #self.station_selection_gui = StationSelection(self._data_manager)
        #self.station_selection_gui.show()

        server_controller = ServerController("ws://127.0.0.1:9000")
        server_controller.register_callback(1, self.login_station)
        server_controller.register_callback(2, self.logout_station)
        server_controller.register_callback(7, self.get_weight_detection)
        server_controller.protocol = ServerSocket
        reactor.listenTCP(9000, server_controller)
        reactor.run()

    def __del__(self):
        with self._camera_process_mutex:
            for cam_index in self._camera_process.keys():
                self.stop_camera(cam_index)

    def start(self):
        LOG_DEBUG("Started StationManager", self._verbose)
        rospy.spin()

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

        with self._camera_process_mutex:
            self._camera_process[camera_id] = subprocess.Popen([self._path_camera_node] + args.split())
            self._transform_process[camera_id] = subprocess.Popen(["roslaunch", self._path_transform_node, f"dev:={camera_id}"])

    def stop_camera(self, camera_id : int):
        LOG_DEBUG(f"Stop Camera {camera_id}", self._verbose)
        with self._camera_process_mutex:
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
        if not self._debug_mode:
            return

        print("Active Cameras: ", self._camera_process.keys())
        for cam_index in self._camera_process.keys():
            self.stop_camera(cam_index)

        self.start_camera(index)

    def kill(self, proc_pid):
        process = psutil.Process(proc_pid)
        for proc in process.children(recursive=True):
            proc.kill()
        process.kill()

    def return_error(self, error_string, code):
        return ResponseAnswer(508, code, {"error" : error_string})

    ### Server Callback functions ###

    def login_station(self, user_id : str, payload : Dict):
        LOG_DEBUG(f"Login {user_id} payload: {payload}", self._verbose)
        if "station" not in payload or "exercise" not in payload:
            return self.return_error("Payload must have a station and an exercise field", 8)

        with self._param_updater_mutex:
            self._param_updater.set_station(payload["station"], True)
            cameras = self._param_updater.get_involved_cameras()

        with self._camera_process_mutex:
            turn_on = cameras - self._camera_process.keys()

        for cam_index in turn_on:
            self.start_camera(cam_index)

        return ResponseAnswer(501, 1, {})

    def logout_station(self, user_id : str, payload : Dict):
        LOG_DEBUG(f"Logout {user_id}, payload : {payload}", self._verbose)
        if "station" not in payload or "exercise" not in payload:
            return self.return_error("Payload must have a station and an exercise field", 8)

        with self._param_updater_mutex:
            self._param_updater.set_station(int(payload["station"]), False)
            cameras = self._param_updater.get_involved_cameras()

        with self._camera_process_mutex:
            turn_off = self._camera_process.keys() - cameras

        for cam_index in turn_off:
            self.stop_camera(cam_index)

        return ResponseAnswer(502, 1, {})

    def get_weight_detection(self, user_id : str, payload : Dict):
        LOG_DEBUG(f"weight detection {user_id}, payload : {payload}", self._verbose)
        if "station" not in payload:
            return self.return_error("Payload must have a exercise field", 8)

        station_id = payload["station"]
        cameras = self._data_manager.get_cameras_of_station(payload["station"])
        if len(cameras) > 0:
            camera_id = list(cameras)[0]
            LOG_DEBUG("Call weight detection service", self._verbose)

            color_msg_list = []
            weight_colors = self._data_manager.get_weight_colors(camera_id, station_id)
            for color_id, color_data in weight_colors.items():
                color_msg_list.append(WeightColor(id=color_id, name=color_data[0], weight=color_data[1],
                    hsv_low=color_data[2], hsv_high=color_data[3], camera_station_id=color_data[4]))

            #result : WeightDetectionResponse = self._ai_weight_detection("image", 2.0, color_msg_list)
            #LOG_DEBUG(f"Weight detection result = {result.weight}kg, response code = {result.response}", self._verbose)
            #return ResponseAnswer(507, 0, {"weight" : result.weight, "probability" : 1})
            return ResponseAnswer(507, 0, {"weight" : 30, "probability" : 1})

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
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
    #station_manager.start()
