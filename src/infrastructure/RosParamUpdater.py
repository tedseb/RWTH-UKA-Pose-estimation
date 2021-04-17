#!/usr/bin/python3
import yaml
import json
import rospy
from std_msgs.msg import String
from backend.msg import StationUsage
import glob

class paramUpdater():
    def __init__(self, cameras):
        #------ channel Name?
        # in hmi.js, config file? 
        rospy.Subscriber('station_usage', StationUsage, self.callback_setStation)
        self._camera_list = cameras
        self._num_cameras = len(cameras)
        self._parameters = {i : {} for i in range(self._num_cameras)}
        self.publisher_pullParam = rospy.Publisher('pullparam', String , queue_size=2)
    def spin(self):
        rospy.spin()

    def callback_setStation(self, msg):
        rospy.loginfo("In Callback", logger_name="my_logger_name")
        station_id = msg.stationID#spot_info_dict["id"]                 #set_station_json["id"]
        station_state = msg.isActive#spot_info_dict["state"]

        print("station_id: ",station_id)
        print("station_state: ",station_state)
        rospy.loginfo(f"station_id: {station_id}", logger_name="RosParam")
        rospy.loginfo(f"station_state: {station_state}", logger_name="RosParam")

        if station_state == True: 
            for i, camera in enumerate(self._camera_list):
                if station_id in camera:
                    self._parameters[i][station_id] = camera[station_id]
        else:
            for stations in self._parameters.values(): 
                if station_id in stations:
                    stations.pop(station_id, None)

        print(self._parameters)
        rospy.loginfo(str(self._parameters), logger_name="RosParam")

        rospy.set_param('param_server', yaml.dump(self._parameters))
        result = rospy.get_param('param_server')
        results_test= yaml.load(result, Loader=yaml.Loader)
        #print(results_test[0])
        self.publisher_pullParam.publish("True")
        

if __name__ == '__main__':
    rospy.init_node('param_updater', anonymous=True)
    
    camera_list = []
    for filename in glob.glob("/home/trainerai/trainerai-core/src/infrastructure/stations/*.yaml"):
        with open(filename) as file:
            station_dic = yaml.load(file, Loader=yaml.Loader)
            camera_list.append(station_dic)
    
    print(camera_list)
    rospy.loginfo(str(camera_list), logger_name="RosParam")
    updater = paramUpdater(camera_list)
    updater.spin()
