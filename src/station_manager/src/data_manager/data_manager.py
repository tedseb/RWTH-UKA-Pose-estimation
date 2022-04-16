from ast import literal_eval as make_tuple
from typing import List, Set, Tuple
from multiprocessing import Lock
import psycopg2
import copy
import pymongo
from .data_manager_interface import DataManagerInterface

class DataManager(DataManagerInterface):
    def __init__(self):
        super().__init__()

        self._camera_station_frames = {}
        self._camera_weight_frames = {}
        self._camera_infos = {} #{id : [names, args]}
        self._station_cameras = {} #{station_id : [camera_id]}
        self._exercises = {}
        self._station_exercises = {}

        try:
            self._connection = psycopg2.connect(user="trainerai",
                                    password="esel1212",
                                    host="127.0.0.1",
                                    port="5432",
                                    database="trainerai_db")
            self._cursor = self._connection.cursor()
        except psycopg2.Error as error:
            raise RuntimeError('Failed to open database') from error

        self._update_frames()
        self._update_camera_infos()
        self._update_station_camera()
        self._load_station_exercises()
        self._mongo_is_on = self._load_exercises()

        self._frame_mutex = Lock()
        self._camera_info_mutex = Lock()
        self._station_camera_mutex = Lock()
        self._postgres_mutex = Lock()

    def __del__(self):
        print("close data manager")
        self._cursor.close()
        self._connection.close()

    def _load_exercises(self):
        mongo_client = pymongo.MongoClient("mongodb://mongoadmin:secret@localhost:27888/?authSource=admin", serverSelectionTimeoutMS=2000)
        if mongo_client is None:
            return False
        db = mongo_client.trainerai
        exercises = db.exercises
        x = exercises.find()
        self._exercises = {}
        for data in x:
            if "description" not in data or "name" not in data:
                continue
            try:
                self._exercises[int(data["name"])] = data["description"]
            except:
                pass
        return True

    def _load_station_exercises(self):
        select_query = 'select "stationId", "exerciseId" from exercises'
        try:
            self._cursor.execute(select_query)
            mobile_records = self._cursor.fetchall()
            exercise_list = {}
            for row in mobile_records:
                print(row)
                if row[0] not in exercise_list:
                    exercise_list[row[0]] = []
                exercise_list[row[0]].append(row[1])
            self._station_exercises = exercise_list
        except psycopg2.Error as error:
            raise RuntimeError("Error while fetching station exercises from PostgreSQL") from error

    def _update_frames(self):
        try:
            select_query = 'SELECT camera_station_mappings.id, camera_station_mappings."cameraId", ' + \
                'camera_station_mappings."stationId", frames.frame_box, frames.type ' + \
                'FROM camera_station_mappings ' + \
                'LEFT JOIN frames ON camera_station_mappings.id = frames."cameraStationMappingId";'

            self._cursor.execute(select_query)
            table = self._cursor.fetchall()

            for row in table:
                if row[4] == 0:
                    frames = self._camera_station_frames
                else:
                    frames = self._camera_weight_frames

                if row[1] not in frames:
                    frames[row[1]] = {}

                if row[3] is not None:
                    box_size = make_tuple("(" + row[3] + ")")
                    frame_list = [box_size[1][0], box_size[1][1], box_size[0][0], box_size[0][1]]
                    if row[4] == 0:
                        frames[row[1]][row[2]] = frame_list
                    elif row[4] == 1:
                        if row[2] not in frames[row[1]]:
                            frames[row[1]][row[2]] = []
                        frames[row[1]][row[2]].append(frame_list)
                else:
                    frames[row[1]][row[2]] = [0, 0, 1280, 720]

            #print(self._camera_station_frames)
            #print(self._camera_weight_frames)
        except psycopg2.Error as error:
            print("Error while fetching frame data from PostgreSQL", error)

    def _update_camera_infos(self):
        try:
            select_query = 'SELECT cameras.id, cameras.name, cameras.type, cameras."typeInfo" ' + \
                "FROM cameras;"

            self._cursor.execute(select_query)
            table = self._cursor.fetchall()

            for row in table:
                self._camera_infos[row[0]] = [row[1], row[2], row[3]]


        except psycopg2.Error as error:
            print("Error while fetching data from PostgreSQL", error)


    def _update_station_camera(self):
        try:
            select_query = 'SELECT id, "cameraId", "stationId" FROM camera_station_mappings'
            self._cursor.execute(select_query)
            mobile_records = self._cursor.fetchall()

            station_cameras = {}
            for row in mobile_records:
                if row[2] not in station_cameras:
                    station_cameras[row[2]] = []
                station_cameras[row[2]].append(row[1])
            self._station_cameras = station_cameras
        except psycopg2.Error as error:
            raise RuntimeError("Error while fetching station/cameras from PostgreSQL") from error

    def get_weight_colors(self, camera_id : int, station_id : int):
        with self._postgres_mutex:
            try:
                select_query = 'SELECT station_weight_colors.id, name, weight, hsv_low, hsv_high, "cameraStationMappingId" ' + \
                'FROM camera_station_mappings ' + \
                'INNER JOIN station_weight_colors ON camera_station_mappings.id=station_weight_colors."cameraStationMappingId" ' + \
                f'WHERE camera_station_mappings."cameraId"={camera_id} and camera_station_mappings."stationId"={station_id};'
                self._cursor.execute(select_query)
                mobile_records = self._cursor.fetchall()
                data = {}
                for record in mobile_records:
                    data[record[0]] = [record[1], record[2], record[3], record[4], record[5]]
                return data
            except psycopg2.Error as error:
                raise RuntimeError("Error while inserting data into weight color") from error

    def get_station_names(self):
        with self._postgres_mutex:
            try:
                select_query = 'SELECT id, name FROM stations;'
                self._cursor.execute(select_query)
                table = self._cursor.fetchall()
                station_names = {}
                for row in table:
                    station_names[row[0]] = row[1]
            except psycopg2.Error as error:
                print("Error while fetching stations data from PostgreSQL", error)
            return station_names

    def get_station_frame_list(self, camer_id : int):
        with self._frame_mutex:
            val = copy.deepcopy(self._camera_station_frames[camer_id])
            return val

    def get_station_frame_lists(self):
        with self._frame_mutex:
            val = copy.deepcopy(self._camera_station_frames)
            return val

    def get_weight_frame_list(self, camer_id : int):
        with self._frame_mutex:
            val = copy.deepcopy(self._camera_weight_frames[camer_id])
            return val

    def get_weight_frame_lists(self):
        with self._frame_mutex:
            val = copy.deepcopy(self._camera_weight_frames)
            return val

    def get_camera_name(self, id : int) -> str:
        with self._camera_info_mutex:
            return self._camera_infos[id][0]

    def get_camera_names(self) -> List[str]:
        with self._camera_info_mutex:
            camera_names = [info[0] for info in self._camera_infos.values()]
            return camera_names

    def get_camera_names_and_indices(self) -> List[Tuple[str, int]]:
        with self._camera_info_mutex:
            camera_indices = [(info[0], index) for index, info in self._camera_infos.items()]
            return camera_indices

    def get_camera_id(self, camera_name : str) -> int:
        with self._camera_info_mutex:
            print(self._camera_infos)
            for index, info in self._camera_infos.items():
                if info[0] == camera_name:
                    return index
            return -1

    def get_camera_type(self, camera_id : int) -> int:
        with self._camera_info_mutex:
            return self._camera_infos[camera_id][1]

    def get_camera_type_info(self, camera_id : int) -> str:
        with self._camera_info_mutex:
            return self._camera_infos[camera_id][2]

    def get_cameras_of_station(self, station_id : int):
        with self._station_camera_mutex:
            return copy.deepcopy(self._station_cameras[station_id])

    def get_stations(self) -> Set:
        station_set = set(s for s in self._station_cameras.keys())
        return station_set

    def is_mongo_on(self):
        return self._mongo_is_on

    def get_exercises(self):
        return copy.deepcopy(self._exercises)

    def get_exercises_on_station(self, station_id : int):
        return copy.deepcopy(self._station_exercises[station_id])
