#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy as rp
import yaml
import json
import operator
import traceback
import os

from rospy_message_converter import message_converter
from threading import Thread
from importlib import import_module
from queue import Queue, Empty, Full

from comparing_system.msg import user_state, user_correction
from backend.msg import Persons
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# ComparingNode imports
from src.joint_adapters.spin import *
from src.Comparator import Comparator
from src.config import *
from src.InterCom import *
from src.FeatureExtraction import *

# db 0 is our (comparing node) database
# TODO: Replace ordinary Redis Queues by ones that store hashes that are keys of dictionaries
redis_connection_pool = redis.ConnectionPool(host='localhost', port=5678, db=0)


class Receiver():
    """
    The Receiver subscribes to the ROS topic that contains joints (or 'skelletons') and puts them into separate queues.
    Each queue corresponds to one spot. Therefore, multiple views of the same spot go into the same queue.
    """
    def __init__(self, spot_queue_load_balancer_class: type(QueueLoadBalancerInterface) = RedisQueueLoadBalancerInterface, spot_queue_interface_class: type(SpotQueueInterface) = RedisSpotQueueInterface):
        # Define a subscriber to retrive tracked bodies
        rp.Subscriber(ROS_JOINTS_TOPIC, Persons, self.callback)
        self.spot_queue_interface = spot_queue_interface_class()

        self.spot_queue_load_balancer = spot_queue_load_balancer_class()
        self.skelleton_adapter = None

    def callback(self, message):
        '''
        This function will be called everytime whenever a message is received by the subscriber.
        It puts the arriving skelletons in the queues for their respective spots, such that we can scale the Comparator.
        '''
        # For every person in the image, sort their data into the correction spot queue in redis

        for p in message.persons:
            p_dict = message_converter.convert_ros_message_to_dictionary(p)
            timestamp = message_converter.convert_ros_message_to_dictionary(message.header.stamp)

            joints_with_timestamp = {'joints': p_dict["bodyParts"], 'timestamp': timestamp}   # Get away from messages here, towards a simple dict

            queue_size = self.spot_queue_interface.enqueue(p.stationID, joints_with_timestamp)

            self.spot_queue_load_balancer.set_queue_size(p.stationID, queue_size)


class Sender(Thread):
    """
    The Sender thread waits for messages in the sending_queue and sends them via ROS as they become available.
    """
    def __init__(self, publisher_topic: str, message_type, redis_sending_queue_name: str, message_queue_interface_class: type(MessageQueueInterface) = RedisMessageQueueInterface):
        super(Sender, self).__init__()
         
        self.publisher_topic = publisher_topic 
        self.redis_sending_queue_name = redis_sending_queue_name
        self.message_queue_interface = message_queue_interface_class()
 
        self.publisher = rp.Publisher(self.publisher_topic, String, queue_size=1000)  

        self.running = True

        self.start()

    def run(self):
        '''
        We publish messages indefinately. If there are none, we wait for the queue to fill and report back every 5 seconds.
        '''
        while(self.running):
            try:
                data = self.message_queue_interface.blocking_dequeue(self.redis_sending_queue_name, timeout=2) # TODO: To not hardcode these two seconds
            except QueueEmpty:
                continue
            except Exception as e:
                if HIGH_VERBOSITY:
                    rp.logerr("Issue getting message from Queue: " + str(self.redis_sending_queue_name))
                    traceback.print_exc()
            try:
                message = json.dumps({'topic': self.publisher_topic, 'data': data})
                self.publisher.publish(message)
                if HIGH_VERBOSITY:
                    rp.logerr("ComparingNode.py sent message: " + str(message))
            except Exception as e:
                raise(e)
                if HIGH_VERBOSITY:
                    rp.logerr("Issue sending message" + str(message) + " to REST API. Error: " + str(e))
                

class SpotInfoHandler():
    """
    This class waits for updates on the spots, such as a change of exercises that the spot.
    Such changes are written into the spot information .json via Redis.
    """
    def __init__(self, spot_info_interface_class: type(SpotInfoInterface) = RedisSpotInfoInterface, message_queue_interface_class: type(MessageQueueInterface) = RedisMessageQueueInterface):
        self.subscriber_expert_system = rp.Subscriber(ROS_EXPERT_SYSTEM_UPDATE_TOPIC, String, self.callback)
        self.spots = dict()
        self.spot_info_interface = spot_info_interface_class()
        self.message_queue_interface = message_queue_interface_class()

    def callback(self, name_parameter_containing_exercises: str):
        spot_update_data = yaml.safe_load(name_parameter_containing_exercises.data)  # TODO: Fit this to API with tamer

        exercise_data = yaml.safe_load(rp.get_param(spot_update_data['parameterServerKey']))
        

        # We need to transform the exercise in this case, so that it matches to old format
        new_stages = list()
        for stage in exercise_data['stages']:
            # Calculate the pose per stage and extract the angles

            joints = stage['skeleton']
            
            pose = {}
            angles = {}
            for index in ownpose_used:
                label = joint_labels[index]
                point = Point()
                # This code currently swaps Y and Z axis, which is how Tamer did this. # TODO: Find defenitive solution to this
                point.x = joints[label]['x']
                point.y = joints[label]['z']
                point.z = joints[label]['y']
                pose[label] = point

            new_angles = list()
            for rule_joints in stage['angles']:
                new_angles.append({'points': rule_joints, 'angle': calculateAngle(rule_joints, pose), 'rules': {}})
            new_stages.append(new_angles)
        exercise_data['stages'] = new_stages
        if EXTRACT_BOUNDARIES:
            exercise_data['boundaries'] = extract_angle_boundaries(exercise_data)
        else:
            exercise_data['boundaries'] = {}
        exercise_data['recording'] = []
                
        now_in_seconds = rp.get_rostime().secs
        new_nanoseconds = rp.get_rostime().nsecs

        spot_queue_key, spot_past_queue_key, spot_info_key = generate_redis_key_names(spot_update_data["stationID"])
        if HIGH_VERBOSITY:
            rp.logerr("Updating info for: " + spot_info_key)
        
        num_deleted_items = self.message_queue_interface.delete(spot_queue_key)
        num_deleted_items += self.message_queue_interface.delete(spot_past_queue_key)
        
        spot_info_dict = {'exercise': exercise_data, 'start_time': now_in_seconds, 'repetitions': 0, 'state': 0}
        self.spot_info_interface.set_spot_info_dict(spot_update_data["stationID"], spot_info_dict)
            

if __name__ == '__main__':
    # initialize ros node
    rp.init_node('comparing_system_node', anonymous=False)

    # TODO: Do not hardcode maxsize
    # Both queues contain dictionaries that can easily converted to YAML to be pusblished via ROS
    user_state_out_queue = Queue(maxsize=QUEUEING_USER_STATE_QUEUE_SIZE_MAX)
    user_correction_out_queue = Queue(maxsize=QUEUEING_USER_INFO_QUEUE_SIZE_MAX)

    # Define a publisher to publish the data for all users to the REST Node
    user_state_sender = Sender(ROS_TOPIC_USER_EXERCISE_STATES, user_state, REDIS_USER_STATE_SENDING_QUEUE_NAME)
    user_correction_sender = Sender(ROS_TOPIC_USER_CORRECTIONS, user_correction, REDIS_USER_INFO_SENDING_QUEUE_NAME)

    # Spawn a couple of Comparator threads
    comparators = []
    for i in range(NUMBER_OF_COMPARATOR_THREADS):
        comparators.append(Comparator())

    receiver = Receiver()

    spot_info_handler = SpotInfoHandler()

    def kill_threads():
        all_threads = comparators + [user_state_sender, user_correction_sender]
        for t in all_threads:
            # TODO: We can not use thread.join() for some reason, envestigate why
            t.running = False
    
    rp.on_shutdown(kill_threads)

    rp.spin()

 