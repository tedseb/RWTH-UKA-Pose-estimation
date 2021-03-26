import yaml
import json
import rospy
from std_msgs.msg import String
import glob

class paramUpdater():
    def __init__(self, cameras):
        #------ channel Name? 
        rospy.Subscriber('test_pub', String, self.callback_setStation)
        self._camera_list = cameras
        self._num_cameras = len(cameras)
        self._parameters = {i : {} for i in range(self._num_cameras)}

    def spin(self):
        rospy.spin()

    def callback_setStation(self, msg):
        set_station_json = json.loads(msg)
        station_id = set_station_json["id"]
        station_state = set_station_json["state"]
        if station_state == True: 
            for i, camera in enumerate(self._camera_list):
                if id in camera:
                    self._parameters[i][id] = camera[id]
        else:
            for stations in self._parameters.values(): 
                if station_id in stations:
                    stations.pop(station_id, None)

        rospy.set_param('param_server', yaml.dump(self._parameters))
        

if __name__ == '__main__':
    camera_list = []
    for filename in glob.glob("./stations/*.yaml"):
        with open(filename) as file:
            station_dic = yaml.load(file, Loader=yaml.Loader)
            camera_list.append(station_dic)
    
    print(camera_list)
    updater = paramUpdater(camera_list)
    updater.spin()
    