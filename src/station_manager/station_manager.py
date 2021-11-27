#!/usr/bin/python3
import json
from typing import Set, Dict
from multiprocessing import Lock
import pathlib
import subprocess
import argparse
import sys
import signal
import psutil
import logy

from src import DataManager, CameraStationController, VideoSelection, StationSelection, TwoWayDict
from src.server import ServerController, ServerSocket, ResponseAnswer
import rospy
import random

from std_msgs.msg import String
from rospy.exceptions import ROSException
from backend.msg import StationUsage, WeightColor, ChannelInfo
from backend.srv import WeightDetection, WeightDetectionResponse, WeightDetectionRequest
from twisted.internet import reactor
import time

DEBUG_STATION_ID = 999

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
    def __init__(self, camera_path, transform_node_path, station_selection_path, verbose=False, debug_frames_ms=0):
        if verbose:
            logy.Logy().basic_config(debug_level=logy.DEBUG, module_name="SM")
        else:
            logy.Logy().basic_config(debug_level=logy.INFO, module_name="SM")

        self._publisher_station_usage = rospy.Publisher('/station_usage', StationUsage , queue_size=5)
        self._publisher_channel_info = rospy.Publisher('/channel_info', ChannelInfo , queue_size=5)
        rospy.Subscriber('user_state', String, self.user_state_callback)
        try:
            rospy.wait_for_service('ai/weight_detection', 3)
            self._ai_weight_detection = rospy.ServiceProxy('ai/weight_detection', WeightDetection)
        except ROSException:
            ("Time out on channel  'ai/weight_detection'")

        self._debug_frames_ms = debug_frames_ms
        self._data_manager = DataManager()
        self._occupied_camera_channels = {}

        self._verbose = verbose
        self._path_camera_node = camera_path
        self._path_transform_node = transform_node_path
        self._path_station_selection = station_selection_path
        self._station_selection_process = subprocess.Popen([self._path_station_selection])

        # Thread Shared Data. Don't Use without Mutex lock!!!
        self.__active_stations = TwoWayDict({}) #Dict[station_id: user_id]
        self.__active_exercises: Dict[str, (int, int, int)] = {} #Dict[user_id: (exercise_id, set_id, repetition)]
        self.__camera_process = {}
        self.__transform_process = {}
        self.__param_updater = CameraStationController(self._data_manager)
        self.__param_updater.add_debug_station(DEBUG_STATION_ID, DEBUG_STATION_ID, [0, 0, 4000, 4000])

        #Mutex
        self._exercise_station_mutex = Lock()
        self._camera_process_mutex = Lock()
        self._param_updater_mutex = Lock()

        self._client_callbacks = {}
        self._server_controller = ServerController("ws://127.0.0.1:3030", self.set_client_callback)
        self._server_controller.register_callback(1, self.login_station_payload)
        self._server_controller.register_callback(2, self.logout_station_payload)
        self._server_controller.register_callback(3, self.start_exercise_payload)
        self._server_controller.register_callback(4, self.stop_exercise_payload)
        self._server_controller.register_callback(7, self.get_weight_detection)
        self._server_controller.protocol = ServerSocket

    def __del__(self):
        with self._camera_process_mutex:
            for cam_index in self.__camera_process.keys():
                self.stop_camera(cam_index)

    def start(self):
        logy.debug("Start StationManager")
        rospy.spin()

    def set_client_callback(self, client_id, callback):
        logy.debug("Register Message Callback")
        self._client_callbacks[client_id] = callback

    def start_camera(self, camera_id: int, debug_station = False):
        logy.debug(f"Start Camera with id {camera_id}")
        if debug_station:
            cam_type = 3
            cam_info = "/home/trainerai/trainerai-core/data/videos/video.mp4"
        else:
            cam_type = self._data_manager.get_camera_type(camera_id)
            cam_info = self._data_manager.get_camera_type_info(camera_id)

        logy.debug(f"Start cam type {cam_type}, on {cam_info}")
        new_channel_id = self.get_new_channel()
        self._occupied_camera_channels[camera_id] = new_channel_id
        new_channel_name = f"/image/channel_{new_channel_id}"

        self._publisher_channel_info.publish(ChannelInfo(new_channel_name, new_channel_id, camera_id, True))

        if cam_type == 0:
            args = f"-y {cam_info} -d {camera_id}"
        if cam_type == 1:
            args = f"-i {cam_info} -d {camera_id}"
        if cam_type == 2:
            args = f"-p {cam_info} -d {camera_id}"
        if cam_type == 3:
            args = f"--disk {cam_info} -d {camera_id}"
        if self._verbose:
            args += " -v"


        args += f' --channel {new_channel_name}'
        args +=  f" --debug-frames {self._debug_frames_ms}"

        with self._camera_process_mutex:
            self.__camera_process[camera_id] = subprocess.Popen([self._path_camera_node] + args.split())
            self.__transform_process[camera_id] = subprocess.Popen(["roslaunch", self._path_transform_node, f"dev:={camera_id}"])

    def stop_camera(self, camera_id: int):
        logy.debug(f"Stop Camera {camera_id}") # We get stuck on this line when using it withing the DataSetRecorder
        with self._camera_process_mutex:
            if camera_id in self.__camera_process:
                self.__camera_process[camera_id].terminate()
                try:
                    self.__camera_process[camera_id].wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.kill(self.__camera_process[camera_id].pid)
                del self.__camera_process[camera_id]

            if camera_id in self.__transform_process:
                self.__transform_process[camera_id].terminate()
                try:
                    self.__transform_process[camera_id].wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.kill(self.__transform_process[camera_id].pid)
                del self.__transform_process[camera_id]

            del self._occupied_camera_channels[camera_id]

    def kill(self, proc_pid):
        process = psutil.Process(proc_pid)
        for proc in process.children(recursive=True):
            proc.kill()
        process.kill()

    def return_error(self, error_string, code):
        return ResponseAnswer(508, code, {"error": error_string})

    def send_repitition(self, user_id: str, repetition: int, exercise: str, set_id: int):
        payload = {
            "repetitions": repetition,
            "exercise": exercise,
            "set_id": set_id
        }
        callback = self._client_callbacks[user_id]
        callback(response_code=509, satus_code=1, payload=payload)

    def get_new_channel(self):
        channel_list = sorted(list(self._occupied_camera_channels.values()))
        new_channel = len(channel_list)
        for i, channel in enumerate(channel_list):
            if i < channel:
                new_channel = i
                break
        return new_channel

    ##################################################################################################################
    #################################### Server Callback functions ###################################################
    ##################################################################################################################

    def login_station_payload(self, user_id: str, payload: Dict):

        logy.debug(f"Login {user_id} payload: {payload}")
        if "station" not in payload:
            return self.return_error("Payload must have a station and an exercise field", 8)

        station_id = int(payload["station"])
        logy.info(f"Login into Station {station_id}")
        return self.login_station(user_id, station_id)

    def login_station(self, user_id: str, station_id: int):
        with self._exercise_station_mutex:
            if station_id in self.__active_stations:
                return ResponseAnswer(501, 4, {})

            if user_id in self.__active_stations:
                return ResponseAnswer(501, 10, {})
        with self._param_updater_mutex:
            self.__param_updater.set_station(station_id, True)
            cameras = self.__param_updater.get_involved_cameras()

        with self._camera_process_mutex:
            turn_on = cameras - self.__camera_process.keys()

        logy.debug(f"Turn on: {turn_on}")
        for cam_index in turn_on:
            self.start_camera(cam_index, station_id==DEBUG_STATION_ID)

        with self._exercise_station_mutex:
            self.__active_stations[user_id] = station_id

        return ResponseAnswer(501, 1, {"station": station_id})

    def logout_station_payload(self, user_id: str, payload: Dict):
        logy.debug(f"Logout {user_id}, payload: {payload}")
        return self.logout_station(user_id)

    def logout_station(self, user_id):
        with self._exercise_station_mutex:
            if user_id not in self.__active_stations:
                return ResponseAnswer(502, 10, {})

            if user_id in self.__active_exercises:
                self.__active_exercises.pop(user_id)

            station_id = self.__active_stations.get(user_id)
            if station_id is None:
                return ResponseAnswer(502, 10, {})
            logy.info(f"Logout from Station {station_id}")

        with self._param_updater_mutex:
            self.__param_updater.set_station(int(station_id), False)
            cameras = self.__param_updater.get_involved_cameras()

        with self._camera_process_mutex:
            turn_off = self.__camera_process.keys() - cameras

        for cam_index in turn_off:
            self.stop_camera(cam_index)

        with self._exercise_station_mutex:
            self.__active_stations.pop(user_id)

        return ResponseAnswer(502, 1, {"station": station_id})

    def start_exercise_payload(self, user_id: str, payload: Dict):
        logy.debug(f"Start exercise {user_id}, payload: {payload}")
        if "station" not in payload or "exercise" not in payload or "set_id" not in payload:
            return self.return_error("Payload must have a station, exercise and set_id field", 8)

        station_id = int(payload["station"])
        exercise_id = int((payload["exercise"]))
        logy.info(f"Start exercise {exercise_id} on Station {station_id}")

        set_id = int((payload["set_id"]))
        answer, _ = self.start_exercise(user_id, station_id, exercise_id, set_id)
        return answer

    def start_exercise(self, user_id: str, station_id: int, exercise_id: int, set_id = 1):
        with self._exercise_station_mutex:
            if self.__active_stations[user_id] != station_id:
                return ResponseAnswer(502, 10, {})

            if user_id in self.__active_exercises:
                return ResponseAnswer(501, 11, {})


        station_usage_hash = str(random.getrandbits(128))
        self._publisher_station_usage.publish(StationUsage(station_id, True , str(exercise_id), station_usage_hash))

        with self._exercise_station_mutex:
            self.__active_exercises[user_id] = (exercise_id, set_id, 0)

        return ResponseAnswer(503, 1, {"station": station_id, "exercise": exercise_id}), station_usage_hash

    def stop_exercise_payload(self, user_id : str, payload : Dict):
        logy.debug(f"Stop exercise {user_id}, payload: {payload}")
        answer, _ = self.stop_exercise(user_id)
        return answer

    def stop_exercise(self, user_id):
        # station_id = int(payload["station"])
        # set_id = int((payload["set_id"]))

        with self._exercise_station_mutex:
            if user_id not in self.__active_stations or user_id not in self.__active_exercises:
                return ResponseAnswer(502, 10, {})
            station_id = self.__active_stations[user_id]
            exercise_data = self.__active_exercises[user_id]
            exercise_id = exercise_data[0]
            set_id = exercise_data[1]

        station_usage_hash = str(random.getrandbits(128))
        self._publisher_station_usage.publish(StationUsage(station_id, False , str(exercise_id), station_usage_hash))
        logy.info(f"Stop exercise {exercise_id} on Station {station_id}")

        with self._exercise_station_mutex:
            self.__active_exercises.pop(user_id)

        return ResponseAnswer(504, 1, {"station": station_id, "exercise": exercise_id, "set_id": set_id}), station_usage_hash

    def get_weight_detection(self, user_id: str, payload: Dict):
        logy.debug(f"weight detection {user_id}, payload: {payload}")
        if "station" not in payload:
            return self.return_error("Payload must have a exercise field", 8)

        station_id = payload["station"]
        cameras = self._data_manager.get_cameras_of_station(payload["station"])
        if len(cameras) > 0:
            camera_id = list(cameras)[0]
            logy.debug("Call weight detection service")

            color_msg_list = []
            weight_colors = self._data_manager.get_weight_colors(camera_id, station_id)
            for color_id, color_data in weight_colors.items():
                color_msg_list.append(WeightColor(id=color_id, name=color_data[0], weight=color_data[1],
                    hsv_low=color_data[2], hsv_high=color_data[3], camera_station_id=color_data[4]))

            result: WeightDetectionResponse = self._ai_weight_detection("image", 2.0, color_msg_list)
            return ResponseAnswer(507, 1, {"weight": result.weight, "probability": 1})

    def user_state_callback(self, msg):
        data = str(msg.data)
        data = json.loads(data)
        station_id = data["data"]["station_id"]
        with self._exercise_station_mutex:
            user_id = self.__active_stations[station_id]
            exercise_data = self.__active_exercises[user_id]
            exercise_id = exercise_data[0]
            set_id = exercise_data[1]
            repetition = exercise_data[2] + 1
            self.__active_exercises[user_id] = (exercise_id, set_id, repetition)
        self.send_repitition(user_id, repetition, exercise_id, set_id)


if __name__ == '__main__':
    rospy.init_node('station_manager', anonymous=False)
    signal.signal(signal.SIGINT, signal_handler)
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="Verbose mode.", action="store_true")
    parser.add_argument("-d", "--debug-frames", default=0, type=int, help="Debug Frame time in ms. At 0 there are no debug frames.")
    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]

    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()

    camera_path = str(pathlib.Path(__file__).absolute().parent.parent) + "/infrastructure/CameraNode.py"
    transform_node_path = str(pathlib.Path(__file__).absolute().parent) + "/launch/static_transform.launch"
    station_selection_path = str(pathlib.Path(__file__).absolute().parent) + "/src/station_selection.py"

    station_manager = StationManager(camera_path, transform_node_path, station_selection_path, verbose=True, debug_frames_ms=args.debug_frames)
    logy.info("Station Manager is Ready")
    reactor.listenTCP(3030, station_manager._server_controller)
    reactor.run()

    ## Use Station Manager Directly ##
    # def repetition_callback(response_code=508, satus_code=2, payload=dict({})):
    #     print("repetitions =", payload["repetitions"])
    #     print("exercise =", payload["exercise"])
    #     print("set id =", payload["set_id"])

    # my_id = "id124"
    # station_manager = StationManager(debug_mode=args.debug, verbose=args.verbose)
    # station_manager.set_client_callback(my_id, repetition_callback)
    # station_manager.login_station(my_id, 2)
    # station_manager.start_exercise(my_id, 2, 105)
    # time.sleep(5)
    # station_manager.stop_exercise(my_id)
    # station_manager.logout_station(my_id)
