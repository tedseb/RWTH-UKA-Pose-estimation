#!/usr/bin/python3
# -*- coding: utf-8 -*-

import json
import operator
import os
import time
import traceback
from importlib import import_module
from queue import Empty, Full, Queue
from random import randint
from sys import maxsize
from threading import Thread
from typing import Any, Dict, List, Tuple

import rospy as rp
import yaml
from geometry_msgs.msg import Point, Vector3
from rospy_message_converter import message_converter
# ComparingNode imports
from src.Comparator import Comparator
from src.config import *
from src.FeatureExtraction import *
from src.InterCom import *
from src.Util import *
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from backend.msg import Persons
from comparing_system.msg import user_correction, user_state

# db 0 is our (comparing node) database
# TODO: Replace ordinary Redis Queues by ones that store hashes that are keys of dictionaries
redis_connection_pool = redis.ConnectionPool(host='localhost', port=5678, db=0)


class Receiver():
    """
    The Receiver subscribes to the ROS topic that contains joints (or 'skelletons') and puts them into separate queues.
    Each queue corresponds to one spot. Therefore, multiple views of the same spot go into the same queue.
    """
    def __init__(self, 
    spot_queue_load_balancer_class: type(QueueLoadBalancerInterface) = RedisQueueLoadBalancerInterface, 
    spot_queue_interface_class: type(SpotQueueInterface) = RedisSpotQueueInterface,
    feature_extractor_class: type(FeatureExtractor) = SpinFeatureExtractor):
        # Define a subscriber to retrive tracked bodies
        rp.Subscriber(ROS_JOINTS_TOPIC, Persons, self.callback)
        self.spot_queue_interface = spot_queue_interface_class()

        self.spot_queue_load_balancer = spot_queue_load_balancer_class()
        self.feature_extractor = feature_extractor_class()

    def callback(self, message: Any) -> None:
        """
        This function will be called everytime whenever a message is received by the subscriber.
        It puts the arriving skelletons in the queues for their respective spots, such that we can scale the Comparator.
        """
        # For every person in the image, sort their data into the correction spot queue in redis

        for p in message.persons:

            array = self.feature_extractor.body_parts_to_ndarray(p.bodyParts)

            p_dict = message_converter.convert_ros_message_to_dictionary(p)

            joints_with_timestamp = {'used_joint_ndarray': array, 'ros_timestamp': message.header.stamp.to_time()}   # Get away from messages here, towards a simple dict

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
                

class SpotMetaDataHandler():
    """This class waits for updates on the spots and communicates them through the its MetaDataInterface."""
    def __init__(self, 
    spot_metadata_interface_class: type(SpotMetaDataInterface) = RedisSpotMetaDataInterface, 
    message_queue_interface_class: type(MessageQueueInterface) = RedisMessageQueueInterface,
    feature_extractor_class: type(FeatureExtractor) = SpinFeatureExtractor):

        self.subscriber_expert_system = rp.Subscriber(ROS_EXPERT_SYSTEM_UPDATE_TOPIC, String, self.callback)
        self.spots = dict()
        self.spot_metadata_interface = spot_metadata_interface_class()
        self.message_queue_interface = message_queue_interface_class()
        self.feature_extractor = feature_extractor_class()

    def callback(self, name_parameter_containing_exercises: str):
        spot_update_data = yaml.safe_load(name_parameter_containing_exercises.data)  # TODO: Fit this to API with tamer

        spot_queue_key, spot_past_queue_key, spot_info_key, spot_state_key, _ = generate_redis_key_names(spot_update_data["stationID"])

        exercise_data = yaml.safe_load(rp.get_param(spot_update_data['parameterServerKey']))
        
        # TODO: In the future: Average recordings? What to do here?
        recording = self.feature_extractor.recording_to_ndarray(exercise_data['recording'])

        feature_of_interest_specification = self.feature_extractor.extract_feature_of_interest_specification_dictionary(exercise_data)

        reference_feature_trajectories = self.feature_extractor.extract_feature_trajectories_from_recordings([recording], feature_of_interest_specification)

        boundaries = self.feature_extractor.extract_boundaries_from_feature_trajectories(reference_feature_trajectories)

        beginning_pose = recording[0]
        exercise_data['beginning_state_dict'] = self.feature_extractor.extract_states(beginning_pose, boundaries, feature_of_interest_specification)

        del exercise_data['stages']
        
        exercise_data['recording'] = recording
        exercise_data['feature_of_interest_specification'] = feature_of_interest_specification
        exercise_data['reference_feature_trajectories'] = reference_feature_trajectories
        exercise_data['boundaries'] = boundaries
        
        spot_info_dict = {'start_time': time.time_ns(), "exercise_data": exercise_data, 'repetitions': 0}

        if HIGH_VERBOSITY:
            rp.logerr("Updating info for: " + spot_info_key)

        num_deleted_items = self.message_queue_interface.delete(spot_queue_key)
        num_deleted_items += self.message_queue_interface.delete(spot_past_queue_key)

        self.spot_metadata_interface.delete(spot_state_key)
        self.spot_metadata_interface.set_spot_info_dict(spot_info_key, spot_info_dict)


if __name__ == '__main__':
    # initialize ros node
    rp.init_node('comparing_system_node', anonymous=False)

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

    spot_info_handler = SpotMetaDataHandler()

    def kill_threads():
        all_threads = comparators + [user_state_sender, user_correction_sender]
        for t in all_threads:
            # TODO: We can not use thread.join() for some reason, envestigate why
            t.running = False
    
    rp.on_shutdown(kill_threads)

    rp.spin()

 