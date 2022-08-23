# Test Modules Station Manager, Object Detection, Metrabs
import time
import sys
import copy
import numpy as np
import psutil
import subprocess
import signal
import roslaunch
import rospy
from typing import List, Tuple, Set, Dict
from std_msgs.msg import Bool
from backend.msg import Bboxes, Persons
from dataclasses import dataclass
sys.path.append('/home/trainerai/trainerai-core/')
from src.station_manager.src.station_manager import StationManager
from src.station_manager.src.data_manager.data_manager_interface import DataManagerInterface
import logy


COMPLETE_MONITORING = True
MONITORING_APPS = ["motion_analysis"]


@dataclass
class CameraFrame:
    camera_id: int
    frame: List[int]  # x, y, w, h


class LocalDataManager(DataManagerInterface):
    def __init__(self):
        self._cameras = {
            0: ["ted_military_press", 0, "sk0khjMgtCU", 43],
            1: ["ted_squats", 0, "u-BAbfHSxN8", 60],
            2: ["ted_deadlifts", 0, "VdByVUstjhA", 65],
            3: ["ted_rowing", 0, "E7gIpHE-JQ4", 36],
            4: ["ted_chess_press", 0, "zg_ks57H4WI", 54],
        }  # [NAME, TYPE, INFO, LENGHT(s)]

        self._stations = {
            0: ["gym_ted_military_press", [CameraFrame(0, [0, 0, 1920, 1080])], 11],
            1: ["gym_ted_squats", [CameraFrame(0, [0, 0, 1920, 1080])], 11],
            2: ["gym_ted_deadlifts", [CameraFrame(0, [0, 0, 1920, 1080])], 11],
            3: ["gym_ted_rowing", [CameraFrame(0, [0, 0, 1920, 1080])], 11],
            4: ["gym_ted_chess_press", [CameraFrame(0, [0, 0, 1920, 1080])], 11],
        }

        self._exercises = {
            229: "Military Press",
            111: "Squats",
            105: "Deadlifts",
            106: "Rowing",
            107: "Chess Press",
        }

        self._station_frame_list = {}
        for station_id, station_info in self._stations.items():
            for camera in station_info[1]:
                if camera.camera_id not in self._station_frame_list:
                    self._station_frame_list[camera.camera_id] = {}
                self._station_frame_list[camera.camera_id][station_id] = camera.frame

    def get_station_names(self) -> Dict[int, str]:
        station_names = {}
        for station_id, station_info in self._stations.items():
            station_names[station_id] = station_info[0]
        return station_names

    def get_station_name(self, station_id) -> str:
        return self._stations[station_id][0]

    def get_station_frame_list(self, camer_id: int) -> List[List[List]]:  # Dict[station] = [x1, y1, x2, y2]
        return copy.deepcopy(self._station_frame_list[camer_id])

    def get_station_frame_lists(self) -> List[List]:  # Dict[camera][station] = [x1, y1, x2, y2]
        return copy.deepcopy(self._station_frame_list)

    def get_camera_name(self, id: int) -> str:
        return self._cameras[id][0]

    def get_camera_names(self) -> List[str]:
        camera_list = [cam[0] for cam in self._cameras]
        return camera_list

    def get_camera_names_and_indices(self) -> List[Tuple[str, int]]:
        camera_list = [(cam[0], cam_id) for cam_id, cam in self._cameras.items()]
        return camera_list

    def get_camera_id(self, camera_name: str) -> int:
        for cam_id, cam_info in self._cameras.items():
            if cam_info[0] == camera_name:
                return cam_id
        return -1

    def get_camera_type(self, camera_id: int) -> int:
        return self._cameras[camera_id][1]

    def get_camera_type_info(self, camera_id: int) -> str:
        return self._cameras[camera_id][2]

    def get_cameras_of_station(self, station_id: int) -> List[int]:
        camera_list = []
        for camera_info in self._stations[station_id][1]:
            camera_list.append(camera_info.camera_id)
        return camera_list

    def get_stations(self) -> Set[int]:
        stations = set()
        for station_id in self._stations.keys():
            stations.add(station_id)
        return stations

    def get_exercises(self) -> Dict[int, str]:  # {id : name}
        return copy.deepcopy(self._exercises)

    def get_exercises_on_station(self, station_id: int):
        raise NotImplementedError

    def is_mongo_on(self):
        return False

    # New
    def get_station_repetitions(self, station_id) -> int:
        return self._stations[station_id][2]


class GymyEnviroment:
    LAUNCH_FILES_ALL = [
        ['logy_backend', 'logy_backend.launch', 'log_level:=debug', 'test:=False'],
        ['object_detection', 'object_detection.launch', 'render:=True'],
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

    def _metrabs_ready_callback(self, msg: Bool) -> None:
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
            if (time.time() - time_now) >= self._metrabs_wait_s:
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

        camera_path = "/home/trainerai/trainerai-core/src/infrastructure/camera_node.py"
        transform_node_path = "/home/trainerai/trainerai-core/src/station_manager/launch/static_transform.launch"
        station_selection_path = "/home/trainerai/trainerai-core/src/station_manager/src/station_selection.py"

        self.data_manager = LocalDataManager()
        self.station_manager = StationManager(
            camera_path,
            transform_node_path,
            station_selection_path,
            data_manager=self.data_manager,
            with_gui=False
        )

        self._metrabs_parent = None
        self._valid = True

    def is_valid(self):
        return self._valid


class ModelTester:
    # 1: Login
    # 2: logout
    # 3: start exercise
    # 4: stop exercise
    # 5: wait time
    TASKS = [
        (1, "user_1", {"station": 0}),
        (3, "user_1", {"station": 0, "exercise": 229, "set_id": 0}),
        (5, 44),
        (4, "user_1", {"station": 0}),
        (2, "user_1", {"station": 0}),
        # (1, "user_1", {"station": 1}),
        # (3, "user_1", {"station": 1, "exercise": 111, "set_id": 0}),
        # (5, 61),
        # (4, "user_1", {"station": 1}),
        # (2, "user_1", {"station": 1}),
    ]

    def __init__(self) -> None:
        """###### Settings ######"""
        self._measure_interval_s = 2
        ######################
        self._env = GymyEnviroment()
        self._env.warm_up(GymyEnviroment.LAUNCH_FILES_ALL)
        self._current_repetitions = 0
        self._shutdown = False
        self._report = {}

    def shutdown(self):
        print("shutdown")
        self._shutdown = True
        self._env.shudown()

    def _user_1_callback(self, response_code=509, satus_code=1, payload=None):
        if payload is None:
            payload = {}
        if response_code == 509:
            if satus_code != 1:
                logy.warn(f"User_1 callback: status code is {satus_code}, '{payload}'")
                return
        self._current_repetitions = payload["repetitions"]

    def _compute_avg(self, old_avg, old_samples, new_value):
        n = old_samples
        y = (n / (n + 1)) * old_avg
        avg = y + (new_value / (n + 1))
        return avg, n + 1

    def get_pid(self, name):
        pids = subprocess.check_output(["pgrep", "-f", name])
        pids = pids.decode("utf-8")
        pids = pids.split("\n")[:-1]
        pids = [int(pid) for pid in pids]
        return pids

    def log_mem(self):
        if not COMPLETE_MONITORING:
            return
        for pname in MONITORING_APPS:
            pids = self.get_pid(pname)
            for pid in pids:
                process = psutil.Process(pid)
                name = process.name().split(".")[0]
                name = f"memory_{name}_{pid}"
                mem_usage = process.memory_info().vms / 1000000000
                logy.log_var(name, mem_usage)

    def start_test(self):
        station_manager = self._env.station_manager
        station_manager.set_client_callback("user_1", self._user_1_callback)

        self._env.start_new_metrabs_session("0", 3)
        active_stations = 0
        for tasks in ModelTester.TASKS:
            time.sleep(0.03)
            if tasks[0] == 1:
                logy.log_str("start_log", f"# Login Station {tasks[2]}")
                logy.info(f"# Login Station {tasks[2]}")
                station_manager.login_station_payload(tasks[1], tasks[2])
                active_stations += 1
                continue
            if tasks[0] == 2:
                logy.log_str("start_log", f"# Logout Station {tasks[2]}")
                logy.info(f"# Logout Station {tasks[2]}")
                station_manager.logout_station_payload(tasks[1], tasks[2])
                active_stations -= 1
                continue
            if tasks[0] == 3:
                self._current_repetitions = 0
                logy.log_str("start_log", f"# Start Exercise {tasks[2]}")
                logy.info(f"# Start Exercise {tasks[2]}")
                station_manager.start_exercise_payload(tasks[1], tasks[2])
                continue
            if tasks[0] == 4:
                logy.log_str("start_log", f"# Stop Exercise {tasks[2]}")
                logy.info(f"# Stop Exercise {tasks[2]}")
                station_manager.stop_exercise_payload(tasks[1], tasks[2])
                repetitions = self._current_repetitions
                station_id = tasks[2]["station"]
                video_repetitions = self._env.data_manager.get_station_repetitions(station_id)
                station_name = self._env.data_manager.get_station_name(station_id)
                # logy.highlighting(f"Repetition {repetitions}/{video_repetitions}")
                self._report[station_name] = (repetitions, video_repetitions)
                continue
            assert tasks[0] == 5
            time.sleep(tasks[1])
        logy.highlighting("###### Report ######")
        for name, repetitions in self._report.items():
            logy.highlighting(f"{name}:", f"{repetitions[0]}/{repetitions[1]}")


if __name__ == '__main__':
    print("Start Test")
    tester = ModelTester()

    def signal_handler(self, signal):
        print("############ SHUTDOWN ##################")
        tester.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    tester.start_test()
