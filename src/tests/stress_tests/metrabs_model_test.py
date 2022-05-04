# Test Modules Station Manager, Object Detection, Metrabs
import time
import sys
import copy
import numpy as np
import nvidia_smi
import psutil
import signal
from threading import Lock

import roslaunch
import rospy
from typing import List, Tuple, Set, Dict
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from backend.msg import ImageData, ChannelInfo, ImageData, Bboxes, Persons, Bboxes
from dataclasses import dataclass
import logy
import cv2
sys.path.append('/home/trainerai/trainerai-core/')
from gymy_tools import ThreadingTimeout
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
            3 : ["military_ted", 0, "O9wJvxWMiBg"],
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
            4 : ["military", [CameraFrame(3, [645, 160, 1018, 701])]],
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

    LAUNCH_FILES_ALL = [
    ['logy_backend', 'logy_backend.launch', 'log_level:=debug', 'test:=False', 'neptune:=test-model_workload'],
    #['station_manager', 'station_manager.launch', 'args:="--without-gui"'],
    #['infrastructure', 'mobile_server.launch'],
    #['metrabs', 'metrabs.launch', 'log_level:=debug'],
    ['object_detection', 'object_detection.launch', 'render:=False'],
    ['motion_analysis', 'motion_analysis.launch', 'log_level:=debug'],
    ['backend', 'SkeletonVisualizationHelper.launch', 'log_level:=debug'],
]

    LAUNCH_FILES_METRABS = [
        ['metrabs', 'metrabs.launch'],
    ]

    def __init__(self) -> None:
        self._valid = False
        self._metrabs_ready = False
        self._metrabs_wait_s = 60
        self._data_manger = None
        rospy.init_node('pytest', anonymous=True)
        rospy.Subscriber('/signals/metrabs_ready', Bool, self._metrabs_ready_callback)

    def shudown(self):
        self._parent.shutdown()
        if self._metrabs_parent is not None:
            self._metrabs_parent.shutdown()

    def __del__(self):
        self._parent.shutdown()
        if self._metrabs_parent is not None:
            self._metrabs_parent.shutdown()

    def _metrabs_ready_callback(self, msg : Bool) -> None:
        if msg:
            self._metrabs_ready = True

    def generate_parent_from_launchfiles(self, launchfiles):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        ros_launch_files = []
        for args in launchfiles:
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(args)[0]
            if len(args) >= 2:
                roslaunch_args = args[2:]
                ros_launch_files.append((roslaunch_file, roslaunch_args))
            else:
                ros_launch_files.append(roslaunch_file)

        parent = roslaunch.parent.ROSLaunchParent(uuid, ros_launch_files)
        return parent

    def start_new_metrabs_session(self, model_name, num_aug):
        if self._metrabs_parent is not None:
            logy.info("Try to Shutdown Metrabs")
            self._metrabs_parent.shutdown()
            logy.info("Done")

        logy.info("Try Start")
        metrabs_launch = copy.deepcopy(GymyEnviroment.LAUNCH_FILES_METRABS)
        metrabs_launch[0].append(f"model:={model_name}")
        metrabs_launch[0].append(f"aug:={num_aug}")

        self._metrabs_parent = self.generate_parent_from_launchfiles(metrabs_launch)
        logy.info("Start Metrabs")
        self._metrabs_parent.start()

        time_now = time.time()
        while not self._metrabs_ready:
            if(time.time() - time_now) >= self._metrabs_wait_s:
                return
            time.sleep(0.05)
        self._metrabs_ready = False
        logy.info("Metrabs Ready")

    def warm_up(self, launchfiles):
        self._parent = self.generate_parent_from_launchfiles(launchfiles)
        self._parent.start()
        time.sleep(2)
        logy.basic_config(debug_level=logy.TEST, module_name="TEST")
        self._data_manger = None
        #time.sleep(3)

        camera_path = "/home/trainerai/trainerai-core/src/infrastructure/CameraNode.py"
        transform_node_path = "/home/trainerai/trainerai-core/src/station_manager/launch/static_transform.launch"
        station_selection_path = "/home/trainerai/trainerai-core/src/station_manager/src/station_selection.py"

        data_manager = LocalDataManager()
        self.station_manager = StationManager(camera_path, transform_node_path, station_selection_path, data_manager=data_manager, with_gui=False)

        self._metrabs_parent = None
        self._valid = True

    def is_valid(self):
        return self._valid

class ModelTester:
    TEST_MODELS = {
        # "metrabs_eff2m_y4" : [[1, 2, 3], "EffNet2M"],
        # "metrabs_eff2s_y4" : [[3, 4, 5], "EffNet2S"],
        # "metrabs_rn101_y4" : [[3, 4, 5], "ResNet101"]
        "metrabs_eff2m_y4" : [[1, 2, 3], "EffNet2M"],
        "metrabs_eff2s_y4" : [[4, 5], "EffNet2S"],
        "metrabs_rn101_y4" : [[4, 5], "ResNet101"]
    }

    def __init__(self) -> None:
        ###### Settings ######
        #self._model_test_time = 10
        self._measure_interval_s = 2
        self._test_station_order = [10, 11, 12, 13]
        self._measurements_per_stations = 10
        self._use_ma = True
        ######################

        self._env = GymyEnviroment()
        self._env.warm_up(GymyEnviroment.LAUNCH_FILES_ALL)
        self._received_bbox_num = 0
        self._received_bbox_avg_s = 0
        self._received_skeleton_num = 0
        self._received_skeleton_avg_s = 0
        self._data_lock = Lock()
        self._shutdown = False

    def shutdown(self):
        self._shutdown = True
        self._env.shudown()

    def _compute_avg(self, old_avg, old_samples, new_value):
        n = old_samples
        y = (n / (n + 1)) * old_avg
        avg = y + (new_value / (n + 1))
        return avg, n+1

    @logy.catch_ros
    def callback_new_bbox(self, msg: Bboxes):
        boxes = np.array(msg.data).reshape(-1, 4)
        elapsed_s = rospy.Time.now().to_sec() - msg.header.stamp.to_sec()
        with self._data_lock:
            if boxes.shape[0] == 0:
                received_bbox_avg_s, received_bbox_num = self._compute_avg(received_bbox_avg_s, received_bbox_num, elapsed_s)
            else:
                for _ in range(boxes.shape[0]):
                    self._received_bbox_avg_s, self._received_bbox_num = self._compute_avg(self._received_bbox_avg_s, self._received_bbox_num, elapsed_s)

    @logy.catch_ros
    def callback_new_skelton(self, msg: Persons):
        elapsed_s = rospy.Time.now().to_sec() - msg.header.stamp.to_sec()
        with self._data_lock:
            for _ in msg.persons:
                self._received_skeleton_avg_s, self._received_skeleton_num = self._compute_avg(self._received_skeleton_avg_s, self._received_skeleton_num, elapsed_s)

    def start_test(self):
        bbox_sub = rospy.Subscriber('bboxes', Bboxes, self.callback_new_bbox, queue_size=50)
        skeleton_sub = rospy.Subscriber('personsJS', Persons, self.callback_new_skelton, queue_size=50)
        station_manager = self._env.station_manager
        for model_name, model_data in ModelTester.TEST_MODELS.items():
            logy.info(f"Start Model {model_name}")

            for num_aug in model_data[0]:
                logy.info(f"Start num_aug '{num_aug}' with model '{model_name}'")
                logy.log_str("start_log", f"### Start model '{model_name}' with num_aug '{num_aug}' ###")
                logy.test(f"### Start model '{model_name}' with num_aug '{num_aug}' ###")
                self._env.start_new_metrabs_session(model_name, num_aug)
                with self._data_lock:
                    self._received_bbox_num = 0
                    self._received_bbox_avg_s = 0
                    self._received_skeleton_num = 0
                    self._received_skeleton_avg_s = 0
                last_time_stamp = time.time()
                for i in range(len(self._test_station_order)):
                    logy.log_str("start_log", f"# Start Station {i}")
                    logy.test(f"# Start Station {i}")
                    station_manager.login_station_payload(f"user_{i}", {"station" : self._test_station_order[i]}) #== SMResponse(501, 1, {"station": i})
                    if self.self._use_ma:
                        station_manager.start_exercise(f"user_{i}", self._test_station_order[i], 111, 1)
                    loop_count = 0
                    while loop_count < self._measurements_per_stations:
                        logy.info(f".... Wait [{loop_count}/{self._measurements_per_stations}] ....")
                        time.sleep(self._measure_interval_s)
                        if self._shutdown:
                            return

                        with self._data_lock:
                            received_bbox_num = self._received_bbox_num
                            received_bbox_avg_s = self._received_bbox_avg_s
                            received_skeleton_num = self._received_skeleton_num
                            received_skeleton_avg_s = self._received_skeleton_avg_s
                            self._received_bbox_num = 0
                            self._received_bbox_avg_s = 0
                            self._received_skeleton_num = 0
                            self._received_skeleton_avg_s = 0
                        duration = time.time() - last_time_stamp
                        last_time_stamp = time.time()
                        received_bbox_fps = (received_bbox_num / duration) / (i + 1)
                        received_skeleton_fps = (received_skeleton_num / duration) / (i + 1)
                        logy.log_var("fps_boxes", received_bbox_fps)
                        logy.log_var("fps_skeleton", received_skeleton_fps)
                        logy.log_var("delay_boxes_ms", received_bbox_avg_s * 1000)
                        logy.log_var("delay_skeleton_ms", received_skeleton_avg_s * 1000)
                        loop_count += 1

                        # Neptune Projekt setzen
                        # Stationen Richtig Starten und Beenden. Für den Anfang, 10 Skunden, dann Station starten. Bis 4 erreicht werden. Dann neues Model laden
                logy.log_str("start_log", f"########################################################### \n")
                logy.test(f"########################################################### \n")
                for i in range(len(self._test_station_order)):
                    if self._use_ma:
                        station_manager.stop_exercise_payload(f"user_{i}", {})
                    station_manager.logout_station_payload(f"user_{i}", {}) # == SMResponse(502, 1, {"station": i})

if __name__ == '__main__':
    print("start")
    tester = ModelTester()
    def signal_handler(self, signal):
        print("############ SHUTDOWN ##################")
        tester.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    tester.start_test()