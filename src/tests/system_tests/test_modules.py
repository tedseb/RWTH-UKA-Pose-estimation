# Test Modules Station Manager, Object Detection, Metrabs
import time
import sys
import copy
import numpy as np
import nvidia_smi
import psutil

import roslaunch
import rospy
from typing import List, Tuple, Set, Dict
from std_msgs.msg import Bool
from backend.msg import ImageData, ChannelInfo, LabelsCameraID, Bboxes, Persons, Bboxes
from dataclasses import dataclass
import pytest
import logy
import cv2
sys.path.append('/home/trainerai/trainerai-core/')

#Station Manager
#sys.path.append('/home/trainerai/trainerai-core/src/station_manager')
from src.station_manager.src.station_manager import StationManager
from src.station_manager.src.server import SMResponse
from src.station_manager.src.data_manager.data_manager_interface import DataManagerInterface


@dataclass
class CameraFrame:
    camera_id : int
    frame : List[int] #x, y, w, h

class LocalDataManager(DataManagerInterface):
    def __init__(self):
        self._cameras = {
            0 : ["orhan_ted_buero", 0, "jX4aJ3_BfNE"],
            1 : ["langhantel_ted", 0, "gTux94rB92w"],
            2 : ["deadlift_ted", 0, "O9wJvxWMiBg"],
            3 : ["military_ted", 0, "Uv9qwcALYKU"],
            10 : ["orhan_ted_buero", 0, "jX4aJ3_BfNE"],
            12 : ["orhan_ted_buero", 0, "jX4aJ3_BfNE"],
            14 : ["orhan_ted_buero", 0, "jX4aJ3_BfNE"],
            16 : ["orhan_ted_buero", 0, "jX4aJ3_BfNE"],
        }

        self._stations = {
            0 : ["Ted", [CameraFrame(0, [249, 68, 650, 708])]],
            1 : ["Orhan", [CameraFrame(0, [1091, 65, 1672, 1058])]],
            2 : ["langhantel", [CameraFrame(1, [562, 100, 871, 693])]],
            3 : ["deadlift", [CameraFrame(2, [645, 160, 1018, 701])]],
            4 : ["military", [CameraFrame(3, [669, 152, 985, 667])]],
            10 : ["Ted_1", [CameraFrame(10, [249, 68, 650, 708])]],
            11 : ["Orhan_1", [CameraFrame(10, [1091, 65, 1672, 1058])]],
            12 : ["Ted_2", [CameraFrame(12, [249, 68, 650, 708])]],
            13 : ["Orhan_2", [CameraFrame(12, [1091, 65, 1672, 1058])]],
            14 : ["Ted_3", [CameraFrame(14, [249, 68, 650, 708])]],
            15 : ["Orhan_3", [CameraFrame(14, [1091, 65, 1672, 1058])]],
            16 : ["Ted_4", [CameraFrame(16, [249, 68, 650, 708])]],
            17 : ["Orhan_4", [CameraFrame(16, [1091, 65, 1672, 1058])]],
        }

        self._exercises = {
            105 : "Deadlifts"
        }

        self._station_frame_list = {}
        for station_id, station_info in self._stations.items():
            for camera in station_info[1]:
                if camera.camera_id not in self._station_frame_list:
                    self._station_frame_list[camera.camera_id] = {}
                self._station_frame_list[camera.camera_id][station_id] = camera.frame

        pass

    def get_station_names(self) -> Dict[int, str]:
        station_names = {}
        for station_id, station_info in self._stations.items():
            station_names[station_id] = station_info[0]
        return station_names

    def get_station_frame_list(self, camer_id : int) -> List[List[List]]: #Dict[station] = [x1, y1, x2, y2]
        return copy.deepcopy(self._station_frame_list[camer_id])

    def get_station_frame_lists(self) -> List[List]: #Dict[camera][station] = [x1, y1, x2, y2]
        return copy.deepcopy(self._station_frame_list)

    def get_camera_name(self, id : int) -> str:
        return self._cameras[id][0]

    def get_camera_names(self) -> List[str]:
        camera_list = [cam[0] for cam in self._cameras]
        return camera_list

    def get_camera_names_and_indices(self) -> List[Tuple[str, int]]:
        camera_list = [(cam[0], cam_id) for cam_id, cam in self._cameras.items()]
        return camera_list

    def get_camera_id(self, camera_name : str) -> int:
        for cam_id, cam_info in self._cameras.items():
            if cam_info[0] == camera_name:
                return cam_id
        return -1

    def get_camera_type(self, camera_id : int) -> int:
        return self._cameras[camera_id][1]

    def get_camera_type_info(self, camera_id : int) -> str:
        return self._cameras[camera_id][2]

    def get_cameras_of_station(self, station_id : int) -> List[int]:
        camera_list = []
        for camera_info in self._stations[station_id][1]:
            camera_list.append(camera_info.camera_id)
        return camera_list

    def get_stations(self) -> Set[int]:
        stations = set()
        for station_id in self._stations.keys():
            stations.add(station_id)
        return stations

    def get_exercises(self) -> Dict[int, str]: #{id : name}
        return copy.deepcopy(self._exercises)

    def get_exercises_on_station(self, station_id : int):
        raise NotImplementedError

    def is_mongo_on(self):
        return False

class GymyEnviroment:

    _LAUNCH_FILES = [
        ['logy_backend', 'logy_backend.launch', 'log_level:=debug', 'test:=True', 'log_tags:=tracing'],
        #['station_manager', 'station_manager.launch', 'args:="--without-gui"'],
        ['infrastructure', 'mobile_server.launch'],
        ['metrabs', 'metrabs.launch', 'log_level:=debug'],
        ['object_detection', 'object_detection.launch', 'render:=False'],
        #['motion_analysis', 'motion_analysis_dev.launch', 'log_level:=debug'],
        ['backend', 'SkeletonVisualizationHelper.launch', 'log_level:=debug'],
    ]

    def __init__(self) -> None:
        self._valid = False
        self._metrabs_ready = False
        self._metrabs_wait_s = 45
        self._data_manger = None
        self.logy = None
        rospy.init_node('pytest', anonymous=True)
        rospy.Subscriber('/signals/metrabs_ready', Bool, self._metrabs_ready_callback)

    def __del__(self):
        self._parent.shutdown()

    def _metrabs_ready_callback(self, msg : Bool) -> None:
        if msg:
            self._metrabs_ready = True

    def warm_up(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        ros_launch_files = []
        for args in self._LAUNCH_FILES:
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(args)[0]
            if len(args) >= 2:
                roslaunch_args = args[2:]
                ros_launch_files.append((roslaunch_file, roslaunch_args))
            else:
                ros_launch_files.append(roslaunch_file)

        self._parent = roslaunch.parent.ROSLaunchParent(uuid, ros_launch_files)
        self._parent.start()

        self._data_manger = None

        time_now = time.time()
        while not self._metrabs_ready:
            if(time.time() - time_now) >= self._metrabs_wait_s:
                print("Metrabs not started")
                return
            time.sleep(0.05)

        print("metrabs ready")
        #time.sleep(3)

        camera_path = "/home/trainerai/trainerai-core/src/infrastructure/CameraNode.py"
        transform_node_path = "/home/trainerai/trainerai-core/src/station_manager/launch/static_transform.launch"
        station_selection_path = "/home/trainerai/trainerai-core/src/station_manager/src/station_selection.py"

        data_manager = LocalDataManager()
        self.station_manager = StationManager(camera_path, transform_node_path, station_selection_path, data_manager=data_manager, with_gui=False)

        self.logy = logy.get_or_create_logger("system_test", logy.TEST, "TEST")
        self._valid = True

    def is_valid(self):
        return self._valid

@pytest.fixture(scope="module")
def ros_env():
    env = GymyEnviroment()
    env.warm_up()
    return env

def _ros_env():
    env = GymyEnviroment()
    env.warm_up()
    return env

class TestCollection:

    def test_station_manager_api(self, ros_env):
        ros_env.logy.test(f"\n####  Station Manager Test  ####", "test")
        assert ros_env.is_valid()
        station_manager = ros_env.station_manager

        command_wait_time_ms = 30
        # Login into no available Station
        assert station_manager.login_station_payload("user_1", {}) == SMResponse(508, 8, {})
        time.sleep(command_wait_time_ms / 1000)
        # Login into no available Station
        assert station_manager.login_station_payload("user_1", {"station" : 100}) == SMResponse(501, 4, {"station": 100})
        time.sleep(command_wait_time_ms / 1000)
        # Login into available Station
        assert station_manager.login_station_payload("user_1", {"station" : 1}) == SMResponse(501, 1, {"station": 1})
        time.sleep(command_wait_time_ms / 1000)
        # Login into second Station with same user
        assert station_manager.login_station_payload("user_1", {"station" : 2}) == SMResponse(501, 10, {"station": 2})
        time.sleep(command_wait_time_ms / 1000)
        # Login into same station with another user
        assert station_manager.login_station_payload("user_2", {"station" : 1}) == SMResponse(501, 4, {"station": 1})
        time.sleep(command_wait_time_ms / 1000)
        # Start without station
        assert station_manager.start_exercise_payload("user_2", {"exercise" : 105, "set_id" : 1}) == SMResponse(508, 8, {})
        time.sleep(command_wait_time_ms / 1000)
        # Start without exercise
        assert station_manager.start_exercise_payload("user_2", {"station" : 1, "set_id" : 1}) == SMResponse(508, 8, {})
        time.sleep(command_wait_time_ms / 1000)
        # Start exercise with wrong user
        assert station_manager.start_exercise_payload("user_2", {"station" : 1, "exercise" : 105, "set_id" : 1}) == SMResponse(503, 10, {"station": 1, "exercise": 105, "set_id": 1})
        time.sleep(command_wait_time_ms / 1000)
        # Start exercise with wrong station
        assert station_manager.start_exercise_payload("user_1", {"station" : 2, "exercise" : 105, "set_id" : 1}) == SMResponse(503, 10, {"station": 2, "exercise": 105, "set_id": 1})
        time.sleep(command_wait_time_ms / 1000)
        # Start exercise correctly
        assert station_manager.start_exercise_payload("user_1", {"station" : 1, "exercise" : 105, "set_id" : 1}) == SMResponse(503, 1, {"station": 1, "exercise": 105, "set_id": 1})
        time.sleep(command_wait_time_ms / 1000)
        # Start exercise a second time
        assert station_manager.start_exercise_payload("user_1", {"station" : 1, "exercise" : 105, "set_id" : 1}) == SMResponse(503, 11, {"station": 1, "exercise": 105, "set_id": 1})
        time.sleep(command_wait_time_ms / 1000)
        # stop exercise wrong user
        assert station_manager.stop_exercise_payload("user_2", {}) == SMResponse(504, 12, {})
        time.sleep(command_wait_time_ms / 1000)
        # stop exercise right
        assert station_manager.stop_exercise_payload("user_1", {}) == SMResponse(504, 1, {"station": 1, "exercise": 105, "set_id": 1})
        time.sleep(command_wait_time_ms / 1000)
        # Logout station wrong user
        assert station_manager.logout_station_payload("user_2", {}) == SMResponse(502, 12, {})
        time.sleep(command_wait_time_ms / 1000)
        # Logout station right
        assert station_manager.logout_station_payload("user_1", {}) == SMResponse(502, 1, {"station": 1})
        time.sleep(command_wait_time_ms / 1000)
        ros_env.logy.test(f"# OK", "test")

    def test_camera(self, ros_env):
        ros_env.logy.test(f"\n####  Camera Test  ####", "test")
        #img = cv2.imread('../data/ted_image.jpg')
        #boxes = [670, 170, 200, 510]
        #assert img is not None
        station_manager = ros_env.station_manager
        received_channel_info_start = False
        received_channel_info_stop = False
        received_channel_info_s = 0
        received_image_s = 0
        received_image_after_channel_s = 0
        received_image_after_sending_s = 0
        received_images = 0
        image_sub = None
        channel_sub = None
        initial_count_image = 0
        INITIAL_COUNT = 5

        @logy.catch_ros
        def callback_new_image(msg: ImageData):
            nonlocal received_images
            nonlocal received_image_s
            nonlocal received_image_after_channel_s
            nonlocal received_image_after_sending_s
            nonlocal initial_count_image

            if initial_count_image <= INITIAL_COUNT:
                initial_count_image += initial_count_image
            if received_images == 0:
                received_image_s = time.time() - received_image_s
                received_image_after_channel_s = time.time() - received_image_after_channel_s

            img_msg = msg.image
            height, width = img_msg.height, img_msg.width
            assert height >= 720
            assert width >= 1280
            elapsed_s = rospy.Time.now().to_sec() - msg.image.header.stamp.to_sec()
            n = received_images
            y = (n / (n + 1)) * received_image_after_sending_s
            avg = y + (elapsed_s / (n + 1))
            received_image_after_sending_s = avg
            received_images += 1

        @logy.catch_ros
        def callback_new_channel(channel_info: ChannelInfo):
            nonlocal image_sub
            nonlocal received_channel_info_s
            nonlocal received_image_after_channel_s
            if channel_info.is_active:
                nonlocal received_channel_info_start
                received_channel_info_s = time.time() - received_channel_info_s
                received_channel_info_start = True
                image_sub = rospy.Subscriber(channel_info.channel_name, ImageData, callback_new_image)
                received_image_after_channel_s = time.time()
            else:
                nonlocal received_channel_info_stop
                received_channel_info_stop = True
                image_sub.unregister()

        received_channel_info_s = time.time()
        received_image_s = time.time()
        channel_sub = rospy.Subscriber('/channel_info', ChannelInfo, callback_new_channel)
        time.sleep(0.01)
        assert station_manager.login_station_payload("user_1", {"station" : 1}) == SMResponse(501, 1, {"station": 1})

        try:
            with logy.TimeOutHandler(6000):
                while received_images == 0:
                    time.sleep(0.01)
                time.sleep(3)
                assert station_manager.logout_station_payload("user_1", {}) == SMResponse(502, 1, {"station": 1})
                while not received_channel_info_stop:
                    time.sleep(0.01)
        except TimeoutError:
            assert not "Time Out"

        channel_sub.unregister()
        ros_env.logy.test(f"# Time for new channel: {received_channel_info_s * 1000:.2f}ms", "test")
        ros_env.logy.test(f"# Time for first image: {received_image_s * 1000:.2f}ms", "test")
        ros_env.logy.test(f"# Time image received after opening channel: {received_image_after_channel_s * 1000:.2f}ms", "test")
        ros_env.logy.test(f"# Time from sending to receive avg: {received_image_after_sending_s * 1000:.2f}ms  N={received_images}", "test")

        assert received_channel_info_s < 0.02
        assert received_image_s < 0.5
        assert received_image_after_sending_s < 0.02
        assert received_channel_info_start
        assert received_channel_info_stop
        assert received_images > 0
        ros_env.logy.test(f"# OK", "test")

    def todo_test_server(self):
        pass

    def todo_image_queue_test(self):
        pass

    def test_object_detection(self, ros_env):
        time.sleep(0.3) #Wait that all ros queues are empty (no remaining messages from old tests)
        ros_env.logy.test(f"\n####  Object Detection Test  ####", "test")
        station_manager = ros_env.station_manager
        received_bbox_s = 0
        received_bbox = False
        bbox_sub = None

        @logy.catch_ros
        def callback_new_bbox(msg: Bboxes):
            nonlocal received_bbox
            nonlocal received_bbox_s
            if not received_bbox:
                received_bbox_s = time.time() - received_bbox_s
                received_bbox = True


        bbox_sub = rospy.Subscriber('bboxes', Bboxes, callback_new_bbox, queue_size=10)
        time.sleep(0.01)
        received_bbox_s = time.time()
        assert station_manager.login_station_payload("user_1", {"station" : 1}) == SMResponse(501, 1, {"station": 1})

        try:
            with logy.TimeOutHandler(10000):
                while not received_bbox:
                    time.sleep(0.01)
        except TimeoutError:
            assert not "Time Out"

        assert station_manager.logout_station_payload("user_1", {}) == SMResponse(502, 1, {"station": 1})
        bbox_sub.unregister()
        ros_env.logy.test(f"# Time from start to first bbox: {received_bbox_s * 1000:.2f}ms", "test")


        assert received_bbox_s < 0.6
        assert received_bbox
        ros_env.logy.test(f"# OK", "test")

# TODO Rendern effizienter

    def test_metrabs_single(self, ros_env):
        time.sleep(0.3) #Wait that all ros queues are empty (no remaining messages from old tests)
        ros_env.logy.test(f"\n####  Metrabs Test  ####", "test")
        station_manager = ros_env.station_manager
        station_manager = ros_env.station_manager
        received_skeleton_s = 0
        received_skeleton = False
        skeleton_sub = None
        channel_sub = None

        @logy.catch_ros
        def callback_new_skelton(msg: Persons):
            print("Skeleton")
            nonlocal received_skeleton
            nonlocal received_skeleton_s
            if not received_skeleton:
                received_skeleton_s = time.time() - received_skeleton_s
                received_skeleton = True

        # @logy.catch_ros
        # def callback_new_channel(channel_info: ChannelInfo):
        #     nonlocal skeleton_sub
        #     if channel_info.is_active:
        #         skeleton_sub = rospy.Subscriber(channel_info.channel_name, Persons, callback_new_skelton)
        #     else:
        #         skeleton_sub.unregister()

        # channel_sub = rospy.Subscriber('/channel_info', ChannelInfo, callback_new_channel)
        # time.sleep(0.01)
        received_skeleton_s = time.time()
        skeleton_sub = rospy.Subscriber('personsJS', Persons, callback_new_skelton)
        assert station_manager.login_station_payload("user_1", {"station" : 1}) == SMResponse(501, 1, {"station": 1})

        try:
            with logy.TimeOutHandler(20000):
                while not received_skeleton:
                    time.sleep(0.01)
        except TimeoutError:
            assert not "Time Out"

        assert station_manager.logout_station_payload("user_1", {}) == SMResponse(502, 1, {"station": 1})
        skeleton_sub.unregister()
        ros_env.logy.test(f"# Time from start to first bbox: {received_skeleton_s * 1000:.2f}ms", "test")


        assert received_skeleton_s < 0.6
        assert received_skeleton
        ros_env.logy.test(f"# OK", "test")

    def todo_test_metrabs_multi_4_one_person(self):
        pass

    def _compute_avg(self, old_avg, old_samples, new_value):
        n = old_samples
        y = (n / (n + 1)) * old_avg
        avg = y + (new_value / (n + 1))
        return avg, n+1

    def _detection_and_metrabs_speed(self, ros_env, station_data, sleep_time_s, step=1): #station_data = [[station, camera, max_avg_ms, max_avg_fps]]
        station_manager = ros_env.station_manager
        station_manager = ros_env.station_manager

        received_skeleton_avg_s = 0
        received_skeleton_num = 0
        initial_counts_skeleton = {} # start with test if each camera sends min. INITIAL_COUNT frames
        initial_time_skeleton = 0

        received_bbox_avg_s = 0
        received_bbox_num = 0
        initial_counts_bbox = {}
        initial_time_bbox = 0

        system_data = [[0, 0], [0, 0] ,[0, 0], [0, 0]] #GPU Util, GPU Mem, CPU, RAM
        skeleton_sub = None
        bbox_sub = None
        INITIAL_COUNT = 10

        nvidia_smi.nvmlInit()
        gp_count = nvidia_smi.nvmlDeviceGetCount()
        #print(f"# GPU count = {gp_count}")
        handle = nvidia_smi.nvmlDeviceGetHandleByIndex(0)

        @logy.catch_ros
        def callback_new_bbox(msg: Bboxes):
            nonlocal initial_counts_bbox
            nonlocal received_bbox_avg_s
            nonlocal received_bbox_num
            nonlocal initial_time_bbox
            camera_id = int(msg.header.frame_id[3:])

            for i, initial_count in initial_counts_bbox.items():
                if initial_count <= INITIAL_COUNT:
                    if i != camera_id:
                        return
                    if initial_count == INITIAL_COUNT:
                        initial_time_bbox = time.time()
                    initial_counts_bbox[i] += 1
                    return
            boxes = np.array(msg.data).reshape(-1, 4)
            elapsed_s = rospy.Time.now().to_sec() - msg.header.stamp.to_sec()
            for _ in range(boxes.shape[0]):
                received_bbox_avg_s, received_bbox_num = self._compute_avg(received_bbox_avg_s, received_bbox_num, elapsed_s)

        @logy.catch_ros
        def callback_new_skelton(msg: Persons):
            nonlocal initial_counts_skeleton
            nonlocal received_skeleton_avg_s
            nonlocal received_skeleton_num
            nonlocal initial_time_skeleton
            camera_id = int(msg.header.frame_id[3:])
            # Before test, all cameras must send min INITIAL_COUNT images
            for i, initial_count in initial_counts_skeleton.items():
                if initial_count <= INITIAL_COUNT:
                    if i != camera_id:
                        return
                    if initial_count == INITIAL_COUNT:
                        initial_time_skeleton = time.time()
                    initial_counts_skeleton[i] += 1
                    return

            elapsed_s = rospy.Time.now().to_sec() - msg.header.stamp.to_sec()
            for _ in msg.persons:
                received_skeleton_avg_s, received_skeleton_num = self._compute_avg(received_skeleton_avg_s, received_skeleton_num, elapsed_s)

            nonlocal system_data
            res_util = nvidia_smi.nvmlDeviceGetUtilizationRates(handle)
            res_mem = nvidia_smi.nvmlDeviceGetMemoryInfo(handle)
            cpu = psutil.cpu_percent()
            ram = psutil.virtual_memory().used
            # print("CPU:", cpu)
            # print("GPU:", res_util.gpu)
            system_data[0][0], system_data[0][1] = self._compute_avg(system_data[0][0], system_data[0][1], res_util.gpu)
            system_data[1][0], system_data[1][1] = self._compute_avg(system_data[1][0], system_data[1][1], res_mem.used)
            if cpu > 0.1:
                system_data[2][0], system_data[2][1] = self._compute_avg(system_data[2][0], system_data[2][1], cpu)
                system_data[3][0], system_data[3][1] = self._compute_avg(system_data[3][0], system_data[3][1], ram)

        def log_and_set_zero(log_str : str, num_stations : int):
            nonlocal received_skeleton_avg_s
            nonlocal received_skeleton_num
            nonlocal received_bbox_avg_s
            nonlocal received_bbox_num
            nonlocal initial_time_skeleton
            nonlocal initial_time_bbox
            nonlocal system_data
            ram_total = psutil.virtual_memory().total
            res_mem = nvidia_smi.nvmlDeviceGetMemoryInfo(handle)
            mib = 1024 * 1024

            time_s = time.time()
            avg_fps_box = (received_bbox_num / (time_s - initial_time_bbox)) / num_stations
            avg_fps_skeleton = (received_skeleton_num / (time_s - initial_time_skeleton)) / num_stations
            ros_env.logy.test(f"# {log_str} time bbox avg: {received_bbox_avg_s * 1000:.2f}ms".ljust(45) + f"N={received_bbox_num}")
            ros_env.logy.test(f"# {log_str} fps bbox avg: {avg_fps_box:.2f}fps".ljust(45) + f"N={received_bbox_num}")
            ros_env.logy.test(f"# {log_str} time skeleton avg: {received_skeleton_avg_s * 1000:.2f}ms".ljust(45) + f"N={received_skeleton_num}")
            ros_env.logy.test(f"# {log_str} fps skeleton avg: {avg_fps_skeleton:.2f}fps".ljust(45) + f"N={received_skeleton_num}")
            ros_env.logy.test(f"# (Average) GPU: {system_data[0][0]:.2f}%, Mem used: {system_data[1][0] / mib:.0f} / {res_mem.total / mib:.0f} MB")
            ros_env.logy.test(f"# (Average) CPU: {system_data[2][0]:.2f}%,      RAM: {system_data[3][0] / mib:.0f} / {ram_total / mib:.0f} MB" )
            ros_env.logy.test(f"# ".ljust(45, '-'))

            received_skeleton_avg_s = 0
            received_skeleton_num = 0
            received_bbox_avg_s = 0
            received_bbox_num = 0
            system_data = [[0, 0], [0, 0] ,[0, 0], [0, 0]]

        bbox_sub = rospy.Subscriber('bboxes', Bboxes, callback_new_bbox, queue_size=10)
        skeleton_sub = rospy.Subscriber('personsJS', Persons, callback_new_skelton)

        for i, data in enumerate(station_data):
            initial_counts_bbox[data[1]] = 0
            initial_counts_skeleton[data[1]] = 0
            assert station_manager.login_station_payload(f"user_{i}", {"station" : data[0]}) == SMResponse(501, 1, {"station": data[0]})
            assert received_bbox_avg_s <= data[2]
            assert received_skeleton_avg_s <= data[3]
            if i % step == (step - 1):
                time.sleep(sleep_time_s)
                log_and_set_zero(f"{i + 1} Station", (i + 1))

        for i, data in enumerate(station_data):
            assert station_manager.logout_station_payload(f"user_{i}", {}) == SMResponse(502, 1, {"station": data[0]})

        skeleton_sub.unregister()
        bbox_sub.unregister()

    def test_detection_and_metrabs_speed_1(self, ros_env):
        time.sleep(0.3) #Wait that all ros queues are empty (no remaining messages from old tests)
        ros_env.logy.test(f"\n####  Detection / Metrabs Speed test (one station / Camera) ####", "test")
        station_data = [
            [1, 0, 0.06, 0.125],
            [2, 1, 0.12, 0.19],
            [3, 2, 0.15, 0.24],
            [4, 3, 0.18, 0.28],
        ]
        self._detection_and_metrabs_speed(ros_env, station_data, 10)
        ros_env.logy.test(f"# OK", "test")

    def test_detection_and_metrabs_speed_2(self, ros_env):
        time.sleep(0.3) #Wait that all ros queues are empty (no remaining messages from old tests)
        ros_env.logy.test(f"\n####  Detection / Metrabs Speed test (two stations / Camera) ####", "test")
        station_data = [
            [10, 10, 0.5, 0.5],
            [11, 10, 0.5, 0.5],
            [12, 12, 0.5, 0.5],
            [13, 12, 0.5, 0.5],
            [14, 14, 0.5, 0.5],
            [15, 14, 0.5, 0.5],
            [16, 16, 0.5, 0.5],
            [17, 16, 0.5, 0.5],
        ]
        self._detection_and_metrabs_speed(ros_env, station_data, 10, 2)
        ros_env.logy.test(f"# OK", "test")

if __name__ == '__main__':

    nvidia_smi.nvmlInit()
    gp_count = nvidia_smi.nvmlDeviceGetCount()
    print(f"# GPU count = {gp_count}")
    handle = nvidia_smi.nvmlDeviceGetHandleByIndex(0)
    time_tmp = time.time()

    res_util = nvidia_smi.nvmlDeviceGetUtilizationRates(handle)
    res_mem = nvidia_smi.nvmlDeviceGetMemoryInfo(handle)
    cpu = psutil.cpu_percent()
    ram = psutil.virtual_memory().used

    ram_total = psutil.virtual_memory().total
    time_tmp = time.time() - time_tmp
    print(f"Time = {time_tmp * 1000}")
    n = 1024 * 1024
    print(f"GPU: {res_util.gpu}%, Mem used: {res_mem.used / n:.0f} / {res_mem.total / n:.0f} MB")
    print(f"CPU: {cpu}%, RAM: {ram / n:.0f} / {ram_total / n:.0f} MB" )


    test = TestCollection()
    env = _ros_env()
    test.test_station_manager_api(env)
