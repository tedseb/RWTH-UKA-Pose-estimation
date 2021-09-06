from ast import literal_eval as make_tuple
from typing import List, Tuple
import psycopg2
import copy

class DataManager():
    def __init__(self):
        super().__init__()

        self._camera_station_frames = {}
        self._camera_weight_frames = {}
        self._camera_infos = {} #{id : [names, args]}

        try:
            self._connection = psycopg2.connect(user="trainerai",
                                    password="esel1212",
                                    host="127.0.0.1",
                                    port="5432",
                                    database="trainerai_db")
            self._cursor = self._connection.cursor()
        except psycopg2.Error as error:
            raise RuntimeError('Failed to open database') from error

        self.update_frames()
        self.update_camera_infos()

    def __del__(self):
        print("close data manager")
        self._cursor.close()
        self._connection.close()

    def update_frames(self):
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

    def get_weight_colors(self, camera_id : int, station_id : int):
        try:
            select_query = 'SELECT station_weight_colors.id, name, weight, hsv_low, hsv_high, cameraStationMappingId ' + \
            'FROM camera_station_mappings ' + \
            'INNER JOIN station_weight_colors ON camera_station_mappings.id=station_weight_colors."cameraStationMappingId" ' + \
            f'WHERE camera_station_mappings."cameraId"={camera_id} and camera_station_mappings."stationId"={station_id};'
            self._cursor.execute(select_query)
            mobile_records = self._cursor.fetchall()
            data = {}
            for record in mobile_records:
                data[record[0]] = [record[1], record[2], record[3], record[4]]
            return data
        except psycopg2.Error as error:
            raise RuntimeError("Error while inserting data into weight color") from error

    def update_camera_infos(self):
        try:
            select_query = 'SELECT cameras.id, cameras.name, cameras.type, cameras."typeInfo" ' + \
                "FROM cameras;"

            self._cursor.execute(select_query)
            table = self._cursor.fetchall()

            for row in table:
                self._camera_infos[row[0]] = [row[1], row[2], row[3]]
            

        except psycopg2.Error as error:
            print("Error while fetching data from PostgreSQL", error)

    def get_station_frame_list(self, camer_id : int):
        val = copy.deepcopy(self._camera_station_frames[camer_id])
        return val

    def get_station_frame_lists(self):
        val = copy.deepcopy(self._camera_station_frames)
        return val

    def get_station_names(self):
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

    def get_weight_frame_list(self, camer_id : int):
        val = copy.deepcopy(self._camera_weight_frames[camer_id])
        return val

    def get_weight_frame_lists(self):
        val = copy.deepcopy(self._camera_weight_frames)
        return val

    def get_camera_name(self, id : int) -> str:
        return self._camera_infos[id][0]

    def get_camera_names(self) -> List[str]:
        camera_names = [info[0] for info in self._camera_infos.values()]
        return camera_names

    def get_camera_names_and_indices(self) -> List[Tuple[str, int]]:
        camera_indices = [(info[0], index) for index, info in self._camera_infos.items()]
        return camera_indices

    def get_camera_id(self, camera_name : str) -> int:
        print(self._camera_infos)
        for index, info in self._camera_infos.items():
            if info[0] == camera_name:
                return index
        return -1

    def get_camera_type(self, camera_id : int) -> int:
        return self._camera_infos[camera_id][1]

    def get_camera_type_info(self, camera_id : int) -> str:
        return self._camera_infos[camera_id][2]
