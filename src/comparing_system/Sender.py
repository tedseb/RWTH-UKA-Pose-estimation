#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Sender Node.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

import json
import traceback
from threading import Thread
from typing import NoReturn

import rospy as rp
import yaml
from src.config import *
from src.FeatureExtraction import *
from src.InterCom import *
from src.Util import *
from std_msgs.msg import String

from comparing_system.msg import user_correction, user_state


class Sender(Thread):
    """
    The Sender thread waits for messages in the sending_queue and sends them via ROS as they become available.
    Published messages are stringified JSON data that is meant leave to the Smartphone App via our RESTful API. 
    """
    def __init__(self, publisher_topic: str, message_type, redis_sending_queue_name: str, message_queue_interface_class: type(MessageQueueInterface) = RedisMessageQueueInterface):
        super(Sender, self).__init__()
         
        self.publisher_topic = publisher_topic 
        self.redis_sending_queue_name = redis_sending_queue_name
        self.message_queue_interface = message_queue_interface_class()
 
        self.publisher = rp.Publisher(self.publisher_topic, String, queue_size=1000)  

        self.running = True

        self.start()

    def run(self) -> NoReturn:
        while(self.running):
            try:
                # We use a blocking dequeue, so that we end up checking if this thread is running regularely
                data = self.message_queue_interface.blocking_dequeue(self.redis_sending_queue_name, timeout=SENDER_NODE_MESSAGE_DEQUEUEING_TIMEOUT)
            except QueueEmpty:
                continue
            except Exception as e:
                if HIGH_VERBOSITY:
                    rp.logerr("Issue getting message from Queue: " + str(self.redis_sending_queue_name))
                    traceback.print_exc()
            try:
                # This is currently how our REST API wants messages to be formed
                message = json.dumps({'topic': self.publisher_topic, 'data': data})
                self.publisher.publish(message)
                if HIGH_VERBOSITY:
                    rp.logerr("ComparingSystem_Sender.py sent message: " + str(message))
            except Exception as e:
                if HIGH_VERBOSITY:
                    rp.logerr("Issue sending message" + str(message) + " to REST API. Error: " + str(e))
         

if __name__ == '__main__':
    # initialize ros node
    rp.init_node('ComparingSystem_Sender', anonymous=False)

    # Define a publisher to publish the data for all users to the REST Node
    user_state_sender = Sender(ROS_TOPIC_USER_EXERCISE_STATES, user_state, REDIS_USER_STATE_SENDING_QUEUE_NAME)
    user_correction_sender = Sender(ROS_TOPIC_USER_CORRECTIONS, user_correction, REDIS_USER_INFO_SENDING_QUEUE_NAME)

    def kill_threads():
        all_threads = [user_state_sender, user_correction_sender]
        for t in all_threads:
            # TODO: We can not use thread.join() for some reason, envestigate why
            t.running = False
    
    rp.on_shutdown(kill_threads)

    rp.spin()

 