#!/usr/bin/python3
import rospy
from std_msgs.msg import String
import logy

class BoxChecker:
    def __init__(self):
        try:
            self._camera_station_boxes = rospy.get_param('station_frames')
        except KeyError:
            self._camera_station_boxes = {}

        rospy.Subscriber('pull_param', String, self._pull_params)

    def _pull_params(self, msg):
        """
        Callback function for reloading the parameter server when a user logs in to TrainerAI.
        """
        self._camera_station_boxes = rospy.get_param('station_frames')

    def check_box_in_station(self, box, camera_id : int) -> int:
        """
        Function that checks if the BBOX is in one of the stations named in the server parameters.
        """
        station_list = self._camera_station_boxes.get(str(camera_id))
        if station_list is None:
            return -1

        for station_id, station_box in station_list.items():
            station_end_x = station_box[0] + station_box[2]
            station_end_y = station_box[1] + station_box[3]
            if (box[0] >= station_box[0]) and (box[1] >= station_box[1]) and (box[2] <= station_end_x) and (box[3] <= station_end_y):
                return int(station_id)

        return -1