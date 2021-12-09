# Test Modules Station Manager, Object Detection, Metrabs
import time
import sys
import copy
import roslaunch
import rospy
from typing import List, Tuple, Set, Dict
from std_msgs.msg import Bool
from backend.msg import ImageData, ChannelInfo, LabelsCameraID, Bboxes, Persons
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
        }

        self._stations = {
            0 : ["Ted", [CameraFrame(0, [249, 68, 650, 708])]],
            1 : ["Orhan", [CameraFrame(0, [1091, 65, 1672, 1058])]],
            2 : ["langhantel", [CameraFrame(1, [562, 100, 871, 693])]],
            3 : ["deadlift", [CameraFrame(2, [645, 160, 1018, 701])]],
            4 : ["military", [CameraFrame(3, [669, 152, 985, 667])]],
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
        ['logy_backend', 'logy_backend.launch', 'log_level:=debug', 'test:=True'],
        #['station_manager', 'station_manager.launch', 'args:="--without-gui"'],
        ['infrastructure', 'mobile_server.launch'],
        ['metrabs', 'metrabs.launch', 'log_level:=debug'],
        ['object_detection', 'object_detection.launch', 'log_level:=debug'],
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
    print("init")
    env = GymyEnviroment()
    print("warm")
    env.warm_up()
    return env

def _ros_env():
    print("init")
    env = GymyEnviroment()
    print("warm")
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
        received_image = False
        received_image_s = 0
        received_image_after_channel_s = 0
        received_image_after_sending_s = 0
        image_sub = None
        channel_sub = None

        @logy.catch_ros
        def callback_new_image(msg: ImageData):
            nonlocal received_image
            nonlocal received_image_s
            nonlocal received_image_after_channel_s
            nonlocal received_image_after_sending_s
            if not received_image:
                received_image_s = time.time() - received_image_s
                received_image_after_channel_s = time.time() - received_image_after_channel_s
                received_image_after_sending_s = rospy.Time.now().to_sec() - msg.image.header.stamp.to_sec()
                img_msg = msg.image
                height, width = img_msg.height, img_msg.width
                assert height >= 720
                assert width >= 1280
                received_image = True

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
            with logy.TimeOutHandler(3000):
                while not received_image:
                    time.sleep(0.01)
                assert station_manager.logout_station_payload("user_1", {}) == SMResponse(502, 1, {"station": 1})
                while not received_channel_info_stop:
                    time.sleep(0.01)
        except TimeoutError:
            assert not "Time Out"

        channel_sub.unregister()
        ros_env.logy.test(f"# Time for new channel: {received_channel_info_s * 1000:.2f}ms", "test")
        ros_env.logy.test(f"# Time for first image: {received_image_s * 1000:.2f}ms", "test")
        ros_env.logy.test(f"# Time image received after opening channel: {received_image_after_channel_s * 1000:.2f}ms", "test")
        ros_env.logy.test(f"# Time from sending in camera node to receive: {received_image_after_sending_s * 1000:.2f}ms", "test")

        assert received_channel_info_s < 0.02
        assert received_image_s < 0.5
        assert received_image_after_sending_s < 0.02
        assert received_channel_info_start
        assert received_channel_info_stop
        assert received_image
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
        def callback_new_bbox(msg: ImageData):
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

    def test_metrabs_single(self, ros_env):
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

    def test_detection_and_metrabs_speed(self):
        pass

    def test_metrabs_multi_4_one_person(self):
        pass

if __name__ == '__main__':
    test = TestCollection()
    env = _ros_env()
    test.test_station_manager_api(env)
