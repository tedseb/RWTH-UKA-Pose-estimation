import yaml
import json
import rospy
from std_msgs.msg import String
import glob

class paramUpdater():
    def __init__(self, cameras):
        #------ channel Name?
        # in hmi.js, config file? 
        rospy.Subscriber('test_pub', String, self.callback_setStation)
        self._camera_list = cameras
        self._num_cameras = len(cameras)
        self._parameters = {i : {} for i in range(self._num_cameras)}

    def spin(self):
        rospy.spin()

    def callback_setStation(self, msg):
        # kommentar von tamer sry
        # node.js StringMsg Type {data: string}
        # python StringMsg ? 
        # take parse this -> msg['data'] into JSON
        result = str(msg).replace("\\", "")
        #print(result)
        #result = "{id': 3, 'state': true}"
        #set_station_json = json.loads(result)
        station_id = 2                  #set_station_json["id"]
        station_state = True #set_station_json["state"]
        if station_state == True: 
            for i, camera in enumerate(self._camera_list):
                if station_id in camera:
                    self._parameters[i][station_id] = camera[station_id]
        else:
            for stations in self._parameters.values(): 
                if station_id in stations:
                    stations.pop(station_id, None)
        print(self._parameters)
        rospy.set_param('param_server', yaml.dump(self._parameters))

        result = rospy.get_param('param_server')
        results_test= yaml.load(result, Loader=yaml.Loader)
        print(results_test[0])
        
        

if __name__ == '__main__':
    rospy.init_node('param_updater', anonymous=True)
    camera_list = []
    for filename in glob.glob("./stations/*.yaml"):
        with open(filename) as file:
            station_dic = yaml.load(file, Loader=yaml.Loader)
            camera_list.append(station_dic)
    
    #print(camera_list)
    updater = paramUpdater(camera_list)
    updater.spin()
    