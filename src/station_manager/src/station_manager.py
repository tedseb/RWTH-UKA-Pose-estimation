import json
from typing import Set, Dict
from multiprocessing import Lock
import pathlib
import subprocess
import argparse
import sys
import signal
import glob
import psutil
#from data_manager import data_manager
import logy

from . import DataManager, CameraStationController, VideoSelection, SMResponse
from gymy_tools import TwoWayDict
import rospy
import random

from std_msgs.msg import String
from rospy.exceptions import ROSException
from backend.msg import StationUsage, WeightColor, ChannelInfo, Bboxes
from backend.srv import WeightDetection, WeightDetectionResponse, WeightDetectionRequest
from twisted.internet import reactor
import time
from gymy_tools import ResetTimer, UniqueQueueDict, ResetTimerHandler

DEBUG_STATION_ID = 999
MAX_STATIONS = 8
CAMERA_CHANNEL_INFO = "/image/channel_{0}"
PERSON_TIME_S = 20
QUEUE_SLOT_EXPIRATION = 40

class ComputerWorkload:
    def __init__(self):
        self.pc_id = 0
        self.gpu_id = 0
        self.cameras = {}
        self.stations = set({})

class StationManager():

    def __init__(self, camera_path, transform_node_path, station_selection_path, data_manager, verbose=False, debug_frames_ms=0, with_gui=True, showroom_mode=False, server_sockets=[]):
        self._publisher_station_usage = rospy.Publisher('/station_usage', StationUsage, queue_size=5)
        self._publisher_channel_info = rospy.Publisher('/channel_info', ChannelInfo, queue_size=5)
        rospy.Subscriber('user_state', String, self.user_state_callback)
        try:
            rospy.wait_for_service('ai/weight_detection', 3)
            self._ai_weight_detection = rospy.ServiceProxy('ai/weight_detection', WeightDetection)
        except ROSException:
            ("Time out on channel  'ai/weight_detection'")

        logy.info('Showroom mode: ' + str(showroom_mode))
        if showroom_mode:
            logy.debug('Start Station Manager in showroom mode')

        self._debug_frames_ms = debug_frames_ms
        self._data_manager = data_manager
        self._occupied_camera_channels = {}

        self._showroom_mode = showroom_mode
        self._with_gui = with_gui
        self._verbose = verbose
        self._path_camera_node = camera_path
        self._path_transform_node = transform_node_path

        if self._with_gui:
            self._path_station_selection = station_selection_path
            self._station_selection_process = subprocess.Popen([self._path_station_selection])

        self._queue_slot_expiration = QUEUE_SLOT_EXPIRATION
        self._queue_timer_handler = ResetTimerHandler(self.queue_timer_callback)
        # Thread Shared Data. Don't Use without Mutex lock!!!
        self.__station_queues = UniqueQueueDict()
        self.__active_stations = TwoWayDict() #Dict[station_id: user_id]
        self.__active_exercises: Dict[str, (int, int, int)] = {} #Dict[user_id: (exercise_id, set_id, repetition)]
        self.__camera_process = {}
        self.__transform_process = {}
        self.__param_updater = CameraStationController(self._data_manager)
        for _id in range(DEBUG_STATION_ID, DEBUG_STATION_ID + 50):
            self.__param_updater.add_debug_station(_id, _id, [0, 0, 4000, 4000])

        #Mutex
        self._exercise_station_mutex = Lock()
        self._camera_process_mutex = Lock()
        self._param_updater_mutex = Lock()

        self._use_person_detection = False
        self.start_person_detection()

        self._client_callbacks = {}
        for socket in server_sockets:
            socket._register_client_callback = self.set_client_callback
            socket.register_callback(1, self.login_station_payload)
            socket.register_callback(2, self.logout_station_payload)
            socket.register_callback(3, self.start_exercise_payload)
            socket.register_callback(4, self.stop_exercise_payload)
            socket.register_callback(5, self.get_available_stations)
            socket.register_callback(7, self.get_weight_detection)
            socket.register_callback(10, self.enqueue_payload)
            socket.register_callback(11, self.dequeue_payload)
            socket.register_callback(13, self.get_queue_state_payload)
            socket.register_callback(16, self.get_queue_list_payload)

    def __del__(self):
        with self._camera_process_mutex:
            for cam_index in self.__camera_process.keys():
                self.stop_camera(cam_index)

    def reset(self):
        # for user_id in
        # self.send_logout_server(user_id)
        logy.info("Reset DigiIsland")
        for user_id in self.__active_stations.get_values():
            self.logout_station(user_id)
            self.send_logout_server(user_id, "reset")

        for user_id, station_id in self.__station_queues.items():
            self.send_logout_queue(user_id, station_id, "reset")
        self.__station_queues.clear()
        self.__station_queues.clear()
        self.__active_exercises.clear()

    def start(self):
        logy.debug("Start StationManager")
        rospy.spin()

    def set_queue_slot_expiration(self, s : int):
        self._queue_slot_expiration = s

    @logy.catch_thread
    def person_time_out(self):
        data = {}
        data["station_id"] = self._next_person_time_out_station
        data["person_active"] = False
        response_json = json.dumps(data)
        self._publisher_persons.publish(response_json)
        debug_time = self._last_person_detected.pop(self._next_person_time_out_station, None)
        if debug_time is None:
            logy.warn("debug_time should not be None")

        if self._last_person_detected:
            self._next_person_time_out_station = min(self._last_person_detected, key=self._last_person_detected.get)
            elapsed_time = time.time() - self._last_person_detected[self._next_person_time_out_station]
            self._person_timer.reset(PERSON_TIME_S - elapsed_time)
        else:
            del self._person_timer
            self._person_timer = None

    @logy.catch_ros
    def callback_bbox(self, box: Bboxes):
        station_id = box.stationID[0]

        if len(box.data) == 0:
            return

        if not self.__param_updater.is_station_valid(station_id):
            return

        if station_id not in self._last_person_detected:
            data = {}
            data["station_id"] = station_id
            data["person_active"] = True
            response_json = json.dumps(data)
            self._publisher_persons.publish(response_json)
            self._next_person_time_out_station = station_id
        self._last_person_detected[station_id] = time.time()

        if self._person_timer is None:
            self._next_person_time_out_station = station_id
            self._person_timer = ResetTimer(PERSON_TIME_S, self.person_time_out)
            self._person_timer.start()
            return

        if station_id != self._next_person_time_out_station:
            return

        self._next_person_time_out_station = min(self._last_person_detected, key=self._last_person_detected.get)
        elapsed_time = time.time() - self._last_person_detected[self._next_person_time_out_station]
        self._person_timer.reset(PERSON_TIME_S - elapsed_time)
        #logy.warn("received boxes")

    def start_person_detection(self):
        self._use_person_detection = True
        self._last_person_detected = {}
        self._person_timer = None
        self._next_person_time_out_station = -1
        rospy.Subscriber('bboxes', Bboxes, self.callback_bbox, queue_size=10)
        self._publisher_persons = rospy.Publisher('/signal/person', String, queue_size=5)

    def set_client_callback(self, client_id, callback):
        logy.debug("Register Message Callback")
        self._client_callbacks[client_id] = callback

    def start_camera(self, camera_id: int, debug_station = False, cam_info=None):
        logy.debug(f"Start Camera with id {camera_id}")
        if debug_station:
            cam_type = 3
            if not cam_info:
                cam_info = "/home/trainerai/trainerai-core/data/videos/video.mp4"
        else:
            cam_type = self._data_manager.get_camera_type(camera_id)
            cam_info = self._data_manager.get_camera_type_info(camera_id)

        logy.debug(f"Start cam type {cam_type}, on {cam_info}")
        new_channel_id = self.get_new_channel()
        self._occupied_camera_channels[camera_id] = new_channel_id

        new_channel_name = CAMERA_CHANNEL_INFO.format(new_channel_id)
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
        new_channel_id = self._occupied_camera_channels[camera_id]
        new_channel_name = CAMERA_CHANNEL_INFO.format(new_channel_id)
        self._publisher_channel_info.publish(ChannelInfo(new_channel_name, new_channel_id, camera_id, False))
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
        return SMResponse(508, code, {"error": error_string})

    def send_repitition(self, user_id: str, repetition: int, exercise: str, set_id: int):
        payload = {
            "repetitions": repetition,
            "exercise": exercise,
            "set_id": int(set_id)
        }
        callback = self._client_callbacks.get(user_id, None)
        if callback is not None:
            callback(response_code=509, satus_code=1, payload=payload)

    def get_new_channel(self):
        channel_list = sorted(list(self._occupied_camera_channels.values()))
        new_channel = len(channel_list)
        for i, channel in enumerate(channel_list):
            if i < channel:
                new_channel = i
                break
        return new_channel

    def fullfill_station_login_condition(self, user_id: str, station_id: int):
        sleeping_time_s = 0.01
        logged_in_user = self.__active_stations.get(station_id, False)
        logged_in_station = self.__active_stations.get(user_id, False)

        if logged_in_user and user_id != logged_in_user:
            logy.debug(f'Another user than "{user_id}" is logged in. Logout user "{logged_in_user}".')
            self.logout_station(logged_in_user)
            time.sleep(sleeping_time_s)

        if logged_in_station and station_id != logged_in_station:
            logy.debug(f'User "{user_id}" is already logged into station "{logged_in_station}". Logout user.')
            self.logout_station(station_id)
            time.sleep(sleeping_time_s)

    def fullfill_exercise_start_condition(self, user_id: str, station_id: int):
        sleeping_time_s = 0.01

        logged_in_station = self.__active_stations.get(user_id, None)

        if logged_in_station != station_id:
            logy.debug(f'User "{user_id}" tries to start an exercise but is not loged into station "{station_id}". Log into right station')
            self.login_station(user_id, station_id)

        running_exercise = self.__active_exercises.get(user_id, None)
        if running_exercise is not None:
            logy.debug(f'User "{user_id}" is doing the exercise "{running_exercise}". Stop exercise')
            self.stop_exercise(user_id)
            time.sleep(sleeping_time_s)

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

    def get_queue_list_payload(self, user_id: str, payload: Dict):
        return self.return_queued_list()

    def return_queued_list(self):
        #station_queues = self.__station_queues.get_queued_numbers()
        return SMResponse(516, 1, {"2":0, "3":0})

    def login_station(self, user_id: str, station_id: int):
        if self._showroom_mode:
            self.fullfill_station_login_condition(user_id, station_id)

        use_queue = False
        with self._exercise_station_mutex:
            if not self.__param_updater.is_station_valid(station_id):
                return SMResponse(501, 4, {"station" : station_id})

            if len(self.__active_stations) >= MAX_STATIONS:
                return SMResponse(501, 3, {"station" : station_id})

            if station_id in self.__active_stations:
                return SMResponse(501, 4, {"station" : station_id})

            if user_id in self.__active_stations:
                return SMResponse(501, 10, {"station" : station_id})

            if self.__station_queues.is_queue_on_key(station_id):
                if self.__station_queues.preview_next_item(station_id) != user_id:
                    return SMResponse(501, 4, {"station" : station_id})
                else:
                    use_queue = True

        #Todo: Check if Station Exist
        with self._param_updater_mutex:
            self.__param_updater.set_station(station_id, True)
            cameras = self.__param_updater.get_involved_cameras()

        with self._camera_process_mutex:
            turn_on = cameras - self.__camera_process.keys()

        logy.debug(f"Turn on: {turn_on}")
        for cam_index in turn_on:
            avis = glob.glob("/home/trainerai/trainerai-core/data/*.avi")
            avis.sort()
            mp4s = glob.glob("/home/trainerai/trainerai-core/data/*.mp4")
            mp4s.sort()
            _videos = avis + mp4s
            cam_info = None if station_id<DEBUG_STATION_ID else _videos[station_id-DEBUG_STATION_ID]
            self.start_camera(cam_index, station_id>=DEBUG_STATION_ID, cam_info=cam_info)

        with self._exercise_station_mutex:
            self.__active_stations[station_id] = user_id

        if use_queue:
            self._queue_timer_handler.remove_timing(station_id)
            self.__station_queues.pop(station_id)
        return SMResponse(501, 1, {"station": station_id})

    def logout_station_payload(self, user_id: str, payload: Dict):
        logy.debug(f"Logout {user_id}, payload: {payload}")
        return self.logout_station(user_id)

    def logout_station(self, user_id):

        running_exercise = self.__active_exercises.get(user_id, None)
        if running_exercise is not None:
            logy.debug(f'User "{user_id}" is doing the exercise "{running_exercise}". Stop exercise.')
            self.stop_exercise(user_id)

        with self._exercise_station_mutex:
            if user_id not in self.__active_stations:
                return SMResponse(502, 12, {})

            station_id = self.__active_stations.get(user_id)
            if station_id is None:
                return SMResponse(502, 10, {})
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

        if self._use_person_detection:
            self._last_person_detected.pop(station_id, None)

        if not self.__station_queues.is_queue_empty(station_id):
            #self.__station_queues.pop(station_id)
            self._queue_timer_handler.add_timing(self._queue_slot_expiration, station_id)
            self.send_queue_notification(station_id)

        return SMResponse(502, 1, {"station": station_id})

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
        if self._showroom_mode:
            self.fullfill_exercise_start_condition(user_id, station_id)

        with self._exercise_station_mutex:
            if user_id not in self.__active_stations:
                return SMResponse(503, 10, {"station": station_id, "exercise": exercise_id, "set_id" : set_id}), None

            if self.__active_stations[user_id] != station_id:
                return SMResponse(503, 10, {"station": station_id, "exercise": exercise_id, "set_id" : set_id}), None

            if user_id in self.__active_exercises:
                return SMResponse(503, 11, {"station": station_id, "exercise": exercise_id, "set_id" : set_id}), None

        station_usage_hash = str(random.getrandbits(128))
        self._publisher_station_usage.publish(StationUsage(station_id, True , str(exercise_id), station_usage_hash))

        with self._exercise_station_mutex:
            self.__active_exercises[user_id] = (exercise_id, set_id, 0)

        return SMResponse(503, 1, {"station": station_id, "exercise": exercise_id, "set_id" : set_id}), station_usage_hash

    def stop_exercise_payload(self, user_id : str, payload : Dict):
        logy.debug(f"Stop exercise {user_id}, payload: {payload}")
        answer, _ = self.stop_exercise(user_id)
        return answer

    def stop_exercise(self, user_id):
        # station_id = int(payload["station"])
        # set_id = int((payload["set_id"]))

        with self._exercise_station_mutex:
            if user_id not in self.__active_stations or user_id not in self.__active_exercises:
                return SMResponse(504, 12, {}), None
            station_id = self.__active_stations[user_id]
            exercise_data = self.__active_exercises.pop(user_id, None)
            exercise_id = exercise_data[0]
            set_id = exercise_data[1]

        station_usage_hash = str(random.getrandbits(128))
        self._publisher_station_usage.publish(StationUsage(station_id, False , str(exercise_id), station_usage_hash))
        logy.info(f"Stop exercise {exercise_id} on Station {station_id}")

        #with self._exercise_station_mutex:
        #self.__active_exercises.pop(user_id, None)

        return SMResponse(504, 1, {"station": station_id, "exercise": exercise_id, "set_id": set_id}), station_usage_hash

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
            return SMResponse(507, 1, {"weight": result.weight, "probability": 1})

    def enqueue_payload(self, user_id: str, payload: Dict):
        logy.debug(f"Enqueue user {user_id}, payload: {payload}")
        if "station" not in payload:
            return self.return_error("Payload must have a station", 8)

        station_id = int(payload["station"])
        answer = self.enqueue(user_id, station_id)
        return answer

    def enqueue(self, user_id: str, station_id : int):
        if self.__station_queues.get_item_queue(user_id) is not None:
            logy.warn("user already loged into queue")
            return SMResponse(510, 14, {})

        if user_id in self.__active_stations:
            logy.warn("user already loged into station")
            return SMResponse(510, 14, {})

        if self.__station_queues.is_queue_full(station_id):
            return SMResponse(510, 13, {})

        self.__station_queues.add(station_id, user_id)

        queue_len = self.__station_queues.get_queue_len(station_id)
        return SMResponse(510, 1, {"queue_slot": queue_len})

    def dequeue_payload(self, user_id: str, payload: Dict):
        logy.debug(f"Dequeue user {user_id}, payload: {payload}")
        answer = self.dequeue(user_id)
        return answer

    def dequeue(self, user_id: str):
        station_id, slot = self.__station_queues.dequeue_item(user_id)
        if station_id is None:
            return SMResponse(511, 1, {})
        # station, queue_slot = self.__station_queues.get_queue_item_info(user_id)
        # if station is not None:
        #     logy.warn(f"item from '{station}', and slot '{queue_slot}' not deleted")
        if slot == 0 and station_id not in self.__active_stations:
            if not self.__station_queues.is_queue_empty(station_id):
                if self.__station_queues.is_queue_empty(station_id):
                    self._queue_timer_handler.remove_timing(station_id)
                else:
                    #self.__station_queues.pop(station_id)
                    self._queue_timer_handler.add_timing(self._queue_slot_expiration, station_id)
                    self.send_queue_notification(station_id)
        return SMResponse(511, 1, {})

    def get_queue_state_payload(self, user_id: str, payload: Dict):
        logy.debug(f"Enqueue user {user_id}, payload: {payload}")
        answer = self.get_queue_state(user_id)
        return answer

    def get_queue_state(self, user_id):
        station, queue_slot = self.__station_queues.get_queue_item_info(user_id)
        if station is None:
            return SMResponse(513, 15, {})
        return SMResponse(513, 1, {"station" : station, "queue_slot" : queue_slot + 1})

    def queue_timer_callback(self, station_id):
        user_id = self.__station_queues.pop(station_id)
        if self.__station_queues.preview_next_item(station_id) is not None:
            self.send_queue_notification(station_id)
            self._queue_timer_handler.add_timing(self._queue_slot_expiration, station_id)

    def send_queue_notification(self, station_id):
        user_id = self.__station_queues.preview_next_item(station_id)
        if user_id is None:
            return
        callback = self._client_callbacks[user_id]
        callback(response_code=512, satus_code=1, payload={"expiration" : self._queue_slot_expiration})

    def send_logout_queue(self, user_id, station_id, reason = "server reset"):
        callback = self._client_callbacks[user_id]
        callback(response_code=514, satus_code=1, payload={"station" : station_id, "reason" : str(reason)})

    def send_logout_server(self, user_id, reason = "server reset"):
        callback = self._client_callbacks[user_id]
        callback(response_code=515, satus_code=1, payload={"reason" : str(reason)})

    def get_available_stations(self, user_id: str, payload: Dict):
        return SMResponse(505, 1, {2: [105, 229, 111]})

    @logy.catch_ros
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
