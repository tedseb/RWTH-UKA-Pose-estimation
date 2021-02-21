#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Shawans Imports
import rospy as rp
from threading import Thread

from comparing_system.msg import user_state, user_correction
from backend.msg import Persons
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# Arturs Imports
from importlib import import_module
from queue import Queue, Empty, Full
import yaml
from src.joint_adapters.spin import *

from src.comparator.Comparator import Comparator

# Artur's Datafields:
# TODO: Read the parameters from ROS launch file


DEFAULT_JOINT_ADAPTER = 'src.joint_adapters.spin'

# TODO: Switch this for a ROS_TOPIC_USER_EXERCISE_STATES ROS_TOPIC
ROS_TOPIC_USER_EXERCISE_STATES = '/user_states'
ROS_TOPIC_USER_CORRECTIONS = '/user_corrections'

ROS_TOPIC_CALLBACK = 'fused_skelleton' # Shawan's topic choice
ROS_SKELLETON_DEFINITION = 'skelleton_name'
NUMBER_OF_COMPARATOR_THREADS = 1


class Receiver():
    def __init__(self, message_in_queue):
        # Define a subscriber to retrive tracked bodies
        rp.Subscriber(ROS_TOPIC_CALLBACK, Persons, self.callback)

        self.message_in_queue = message_in_queue
        self.skelleton_adapter = None

        rp.spin()

    def callback(self, message):
        '''
        This function will be called everytime whenever a message is received by the subscriber.
        It puts the arriving message in a message queue, such that we can scale the Comparator.
        '''
        try:
            from_ros_parameter_server_current_exercise = yaml.safe_load(rp.get_param('exercise'))
        except KeyError as e:
            print("No exercise set under rostopic 'exercise', maybe the expert system is not running or ROS is not properly set up?")
            return

        data = {'current_exercise': from_ros_parameter_server_current_exercise, 'message': message}

        try:
            self.message_in_queue.put_nowait(data)
        except Full:
            rp.logerr("ComparingNode message queue is full. Comparator thread(s) not dequeuing fast enough from message queue. Consider adding more Comparator threads.")
        

class Sender(Thread):
    def __init__(self, publisher_topic, message_type,queue):
        super(Sender, self).__init__()
        self.publisher = rp.Publisher(publisher_topic, message_type, queue_size=1000)      
        self.queue = queue

        self.start()

    def run(self):
        '''
        We publish messages indefinately. If there are none, we wait for the queue to fill and report back every second.
        '''
        while(True):
            if not rp.is_shutdown():
                try:
                    message = self.queue.get(timeout=1)
                    self.publisher.publish(message)
                except Empty:
                    print("Nothing to send.")
                except Exception as e:
                    print("Issue sending message" + str(message))
            else:
                # If rospy is shut down, exit this thread
                return


if __name__ == '__main__':
    # initialize ros node
    rp.init_node('comparing_system_node', anonymous=False)

    # TODO: Maybe get this ready for shared handling
    # TODO: Do not hardcode maxsize
    message_in_queue = Queue(maxsize=10)
    user_state_out_queue = Queue(maxsize=1000)
    user_correction_out_queue = Queue(maxsize=1000)

    # Define a publisher to publish the 3D skeleton of multiple people
    user_state_sender = Sender(ROS_TOPIC_USER_EXERCISE_STATES, user_state, user_state_out_queue)
    user_correction_sender = Sender(ROS_TOPIC_USER_CORRECTIONS, user_correction, user_correction_out_queue)

    # Spawn a couple of Comparator threads
    # TODO: Possibly distribute for load balancing
    comparators = []
    for i in range(NUMBER_OF_COMPARATOR_THREADS):
        comparators.append(Comparator(message_in_queue, user_state_out_queue, user_correction_out_queue))

    Receiver(message_in_queue)
    
    for i in comparators:
        i.run()
