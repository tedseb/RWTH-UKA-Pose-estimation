#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy as rp
import yaml
import json
import operator

from threading import Thread
from importlib import import_module
from queue import Queue, Empty, Full
from collections import OrderedDict

from comparing_system.msg import user_state, user_correction
from backend.msg import Persons
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# ComparingNode imports
from src.joint_adapters.spin import *
from src.Comparator import Comparator
from src.config import *
from src.Queueing import *

# db 0 is our (comparing node) database
# TODO: Replace ordinary Redis Queues by ones that store hashes that are keys of dictionaries
redis_connection_pool = redis.ConnectionPool(host='localhost', port=5678, db=0)


class Receiver():
    """
    The Receiver subscribes to the ROS topic that contains joints (or 'skelletons') and puts them into separate queues.
    Each queue corresponds to one spot. Therefore, multiple views of the same spot go into the same queue.
    """
    def __init__(self, message_queue_load_order, spot_queue_interface_class: type(SpotQueueInterface) = RedisSpotQueueInterface):
        # Define a subscriber to retrive tracked bodies
        rp.Subscriber(ROS_JOINTS_TOPIC, Persons, self.callback)
        self.spot_queue_interface = spot_queue_interface_class()

        self.message_queue_load_order = message_queue_load_order
        self.skelleton_adapter = None

    def callback(self, message):
        '''
        This function will be called everytime whenever a message is received by the subscriber.
        It puts the arriving skelletons in the queues for their respective spots, such that we can scale the Comparator.
        '''

        # TODO: Change all these ROS messages to python dicts

        # For every person in the image, sort their data into the correction spot queue in redis
        for p in message.persons:
            redis_spot_key = 'spot #' + str(p.stationID)

            joints_with_timestamp = {'joints': p.bodyParts, 'timestamp': message.header.stamp}   # Get away from messages here, towards a simple dict

            queue_size = self.spot_queue_interface.enqueue(redis_spot_key, joints_with_timestamp)



class Sender(Thread):
    """
    The Sender thread waits for messages in the sending_queue and sends them via ROS as they become available.
    """
    def __init__(self, publisher_topic: str, message_type, redis_sending_queue_name: str, message_queue_interface_class: type(MessageQueueInterface) = RedisMessageQueueInterface):
        super(Sender, self).__init__()
        self.publisher = rp.Publisher(publisher_topic, String, queue_size=1000)    
        self.redis_sending_queue_name = redis_sending_queue_name
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)  
        self.message_queue_interface = message_queue_interface_class()

        self.running = True

        self.start()

    def run(self):
        '''
        We publish messages indefinately. If there are none, we wait for the queue to fill and report back every 5 seconds.
        '''
        while(self.running):
            try:
                message = self.message_queue_interface.blocking_dequeue(self.redis_sending_queue_name, timeout=2) # TODO: To not hardcode these two seconds
            except Exception as e:
                rp.logerr("Issue getting message from Queue: " + str(self.redis_sending_queue_name) + ", Exception: " + str(e))
            try:
                # Unpack message dict and error out if it contains bad fields
                self.publisher.publish(json.dumps(message))
                rp.loginfo("ComparingNode.py sent message: " + str(message))
            except Exception as e:
                raise(e)
                rp.logerr("Issue sending message" + str(message) + " to REST API. Error: " + str(e))
                


class SpotInfoHandler():
    """
    This class waits for updates on the spots, such as a change of exercises that the spot.
    Such changes are written into the spot information .json via Redis.
    """
    def __init__(self):
        self.subscriber = rp.Subscriber(ROS_EXERCISES_CHANGE_TOPIC, String, self.callback)
        self.spots = dict()
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)        

    def callback(self, name_parameter_containing_exercises):
        last_spots = self.spots
        self.spots = yaml.safe_load(rp.get_param(name_parameter_containing_exercises.data)) # TODO: Fit this to API with tamer
        
        now = rp.get_rostime()
        for k, v in self.spots.items():
            exercise_and_start_time = {'exercise': v, 'start_time': now}
            if last_spots.get(k) == v:
                # Each spot has its own queue as well as an info .json object
                redis_spot_queue_key = 'spot #' + str(k) + ':queue'
                redis_spot_info_key = 'spot #' + str(k) + ':info' 
                redis_spot_past_queue_key = redis_spot_queue_key + "_past"
                # Use keys to delete queues and update info on spots
                self.redis_connection.set(redis_spot_info_key, yaml.dump(exercise_and_start_time))
                num_deleted_items = self.redis_connection.delete(redis_spot_queue_key)
                num_deleted_items += self.redis_connection.delete(redis_spot_past_queue_key)
                rp.loginfo("Deleted " + str(num_deleted_items) + " from " + redis_spot_queue_key + " due to an exercise change at ROS time " + str(now))
                

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

    # Put queue lengths with queue names here
    message_queue_load_order = OrderedDict()

    # Spawn a couple of Comparator threads
    comparators = []
    for i in range(NUMBER_OF_COMPARATOR_THREADS):
        comparators.append(Comparator(message_queue_load_order))

    receiver = Receiver(message_queue_load_order)

    spot_info_handler = SpotInfoHandler()

    def kill_threads():
        all_threads = comparators + [user_state_sender, user_correction_sender]
        for t in all_threads:
            # TODO: We can not use thread.join() for some reason, envestigate why
            t.running = False

    rp.on_shutdown(kill_threads)

    # TEST CODE, REMOVE AS SOON A TAMER PUBLISHES EXERCISES
    publisher = rp.Publisher(ROS_EXERCISES_CHANGE_TOPIC, String, queue_size=10)
    message = 'exercise'
    import time
    for i in range(3):
        time.sleep(0.9)
        publisher.publish(message)

    rp.spin()

 