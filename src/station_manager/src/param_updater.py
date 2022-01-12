#!/usr/bin/python3
import yaml
import rospy
from std_msgs.msg import String
from ast import literal_eval as make_tuple
from .data_manager import DataManager
from typing import Set, Dict, List
import logy
#TODO camera_station to station_camera

class CameraStationController():
    def __init__(self, data_manager : DataManager):
        self._data_manager = data_manager
        self._camera_station_frames : Dict[int, Dict[int, List]] = {}
        self._camera_weight_frames = {}
        self._station_frame_parameters = {}
        self._weight_frame_parameters = {}
        self._publisher_pull_param = rospy.Publisher('pull_param', String , queue_size=2)

        self._camera_station_frames = self._data_manager.get_station_frame_lists()
        self._stations = self._data_manager.get_stations()
        logy.warn(self._stations)
        #self._camera_weight_frames = self._data_manager.get_weight_frame_lists()
        #for camera_id, station_dict in frames.items():
            #self._camera_list[camera_id] = station_dict
        logy.debug(f"Camera list: {self._camera_station_frames}")

    def is_station_valid(self, station_id) -> bool:
        if station_id in self._stations:
            return True
        logy.warn(str(self._stations))
        return False

    def add_debug_station(self, station_id, camera_id, frame):
        if camera_id not in self._camera_station_frames:
            self._camera_station_frames[camera_id] = {}
        self._camera_station_frames[camera_id][station_id] = frame
        self._stations.add(station_id)

    def get_involved_cameras(self) -> Set[int]:
        active_cameras = {int(cam) for cam in self._station_frame_parameters}
        return active_cameras

    def set_station(self, station_id : int, is_active : bool):
        self.set_or_delete_frame_param(station_id, is_active, self._camera_station_frames, self._station_frame_parameters)

        logy.debug(f"Station Frame Parameters: {self._station_frame_parameters}")
        rospy.set_param('station_frames', self._station_frame_parameters)
        rospy.set_param('weight_frames', {"empty" : 0})

        self._publisher_pull_param.publish("True")

    def set_or_delete_frame_param(self, station_id : int, station_state : bool, frame_list, parameter_to_set):
        if station_state:
            for camera_id, stations in frame_list.items():
                if station_id in stations:
                    if str(camera_id) not in parameter_to_set:
                        parameter_to_set[str(camera_id)] = {}
                    parameter_to_set[str(camera_id)][str(station_id)] = stations[station_id]
        else:
            key_to_delete = []
            for key, stations in parameter_to_set.items():
                stations.pop(str(station_id), None)
                if not stations:
                    key_to_delete.append(str(key))
            for key in key_to_delete:
                parameter_to_set.pop(key, None)
