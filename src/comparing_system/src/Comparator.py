"""
This file contains the Comparator. 
"""


import threading
import math
import rospy as rp
import redis
import yaml

from importlib import import_module
from threading import Thread

from src.joint_adapters.spin import *
from src.config import *
from src.Queueing import *
from src.LegacyCompaing import compare_legacy

from traceback import print_exc


class NoJointsAvailable(Exception):
    pass

class NoSpotInfoAvailable(Exception):
    pass

class Comparator(Thread):
    """
    The Comparator can be scaled horizontally. It pops data from inbound spot queues and calculates and metrics which are put into outbound message queues.
    """
    def __init__(self, spot_queue_load_balancer_class: type(QueueLoadBalancerInterface) = RedisQueueLoadBalancerInterface, message_out_queue_interface_class: type(MessageQueueInterface) = RedisMessageQueueInterface, spot_queue_interface_class: type(SpotQueueInterface) = RedisSpotQueueInterface):
        super(Comparator, self).__init__()

        self.spot_queue_load_balancer = spot_queue_load_balancer_class()
        self.message_out_queue_interface = message_out_queue_interface_class()
        self.spot_queue_interface = spot_queue_interface_class()
        
        self.redis_connection = redis.StrictRedis(connection_pool=redis_connection_pool)

        # This can be set to false by an external entity to stop the loop from running
        self.running = True

        self.start()

    def run(self):
        """
        This is our main threading loop. We devide it into three parts for a  better overview:
            * Getting data from the queues with self.get_data()
            * Comparing the "is" and "should be" data for the joints with self.compare()
            * Putting data back into sending queues with self.send_info()
        """
        while(self.running):
            try:
                # Get data from queues
                spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list = self.get_data()
                # Compare joints with expert system data
                comparison_info = self.compare(spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list)
                # Send info back back to outgoing message queue and back into the ROS system
                self.send_info(comparison_info, spot_info_dict)
            except QueueEmpty:
                continue
            except Exception as e:
                print_exc() 
                rp.logerr("Error sending data in the comparator: " + str(e))    
            
    def get_data(self):
        """
        This method gets information from inbound queues, checks for errors like empty queues and returns information if any is present
        """
        # Fetch all data that is needed for the comparison:
        redis_spot_key = self.spot_queue_load_balancer.get_queue_key()

        return self.spot_queue_interface.dequeue(redis_spot_key)
        
    def send_info(self, comparison_info: dict, spot_info_dict: dict):
        """
        This method packs information that occurs in the process of analyzing the joint- and exercise-data into messages and enques them into an sending queue.
        """

        updated_repetitions, correction, center_of_body = comparison_info

        if correction != None and SEND_CORRETIONS:
            user_correction_message = {
                'user_id': 0,
                'repetition': updated_repetitions,
                'positive_correction': False,
                'display_text': correction
            }
            self.message_out_queue_interface.enqueue(REDIS_USER_INFO_SENDING_QUEUE_NAME, user_correction_message)

        if updated_repetitions != None:
            user_state_message = {
                'user_id': 0,
                'current_exercise_name': spot_info_dict.get('exercise').get('name'),
                'repetitions': updated_repetitions,
                'seconds_since_last_exercise_start': (rp.Time.now() - spot_info_dict.get('start_time')).to_sec(),
                'milliseconds_since_last_repetition': 0,
                'repetition_score': 100,
                'exercise_score': 100
            }
            self.message_out_queue_interface.enqueue(REDIS_USER_STATE_SENDING_QUEUE_NAME, user_state_message)
        
    def compare(self, spot_info_dict: dict, past_joints_with_timestamp_list: list, joints_with_timestamp: list, future_joints_with_timestamp_list: dict):
        if LEGACY_COMPARING:
            return compare_legacy(spot_info_dict, past_joints_with_timestamp_list, joints_with_timestamp, future_joints_with_timestamp_list)
        else:
            raise NotImplementedError
