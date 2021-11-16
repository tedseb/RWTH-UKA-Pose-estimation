#!/usr/bin/python3
import rospy
from std_msgs.msg import String
import logy

class getBoxesInStation:
    def __init__(self):
        try:
            self.station_boxes = rospy.get_param('station_frames')
        except KeyError:
            self.station_boxes = {}

        try:
            self.weight_boxes = rospy.get_param('weight_frames')
        except KeyError:
            self.weight_boxes = {}

        rospy.Subscriber('pull_param', String, self.setStation)

    def setStation(self, msg):
        """
        Callback function for reloading the parameter server when a user logs in to TrainerAI.
        """

        self.station_boxes = rospy.get_param('station_frames')
        self.weight_boxes = rospy.get_param('weight_frames')
        logy.debug("New Station Param is set in pull_param: " + str(self.station_boxes))

        #print("SCHEDULER STATIONS: ", self.station_boxes)
        #print("SCHEDULER WEIGHT FRAMES: ", self.station_boxes)


    def run_BoxStationChk(self, box, frame_id, station_check):
        """
        Function that checks if the BBOX is in one of the stations named in the server parameters.
        """
        frame_id_str = str(frame_id)
        if station_check:
            if frame_id_str in self.station_boxes:
                for station_id, station_box in self.station_boxes[frame_id_str].items():
                    station_end_x= station_box[0]+station_box[2]
                    station_end_y= station_box[1]+station_box[3]
                    if (box[0]>=station_box[0]) and (box[1]>=station_box[1]) and (box[2]<=station_end_x) and (box[3]<=station_end_y): #X0 Y0 X1 Y1
                        return True, int(station_id)
                return False,0
            else:
                return False,0
        else:
            return True,0
