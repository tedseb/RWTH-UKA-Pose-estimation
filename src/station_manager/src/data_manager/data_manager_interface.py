from typing import List, Set, Tuple, Dict

class DataManagerInterface():
    def __init__(self):
        pass

    def get_weight_colors(self, camera_id : int, station_id : int):
        raise NotImplementedError

    def get_station_names(self) -> Dict[int, str]:
        raise NotImplementedError

    def get_station_frame_list(self, camer_id : int) -> List[List[List]]: #Dict[station] = [x1, y1, x2, y2]
        raise NotImplementedError

    def get_station_frame_lists(self) -> List[List]: #Dict[camera][station] = [x1, y1, x2, y2]
        raise NotImplementedError

    def get_weight_frame_list(self, camer_id : int):
        raise NotImplementedError

    def get_weight_frame_lists(self):
        raise NotImplementedError

    def get_camera_name(self, id : int) -> str:
        raise NotImplementedError

    def get_camera_names(self) -> List[str]:
        raise NotImplementedError

    def get_camera_names_and_indices(self) -> List[Tuple[str, int]]:
        raise NotImplementedError

    def get_camera_id(self, camera_name : str) -> int:
        raise NotImplementedError

    def get_camera_type(self, camera_id : int) -> int:
        raise NotImplementedError

    def get_camera_type_info(self, camera_id : int) -> str:
        raise NotImplementedError

    def get_cameras_of_station(self, station_id : int) -> List[int]:
        raise NotImplementedError

    def get_stations(self) -> Set[int]:
        raise NotImplementedError

    def get_exercises(self) -> Dict[int, str]: #{id : name}
        raise NotImplementedError

    def get_exercises_on_station(self, station_id : int):
        raise NotImplementedError

    def is_mongo_on(self) -> bool:
        raise NotImplementedError
