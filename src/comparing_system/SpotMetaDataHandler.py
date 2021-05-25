#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Spot MetaDataHandler Node.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

import time
from typing import NoReturn

import rospy as rp
import yaml
from src.config import *
from src.FeatureExtraction import *
from src.InterCom import *
from src.Util import *
from std_msgs.msg import String


class SpotMetaDataHandler():
    """Waits for updates on the usage of spots and communicates them through the a SpotMetaDataInterface.
    
    The SpotMetaDataHandler waits for stringified YAMLs from the HMI. This data includes recordings of experts and further
    data regarding the exercise. It uses a feature extractor to extract information that is found in the recording.
    These includes expecially, but is not limited to, trajectories of features of interest. For example the trajectory of
    the angle of the left and right knee.

    The extracted features are set in a SpotMetaDataInterface and become avaiblable to the Comparator threads.
    Furthermore, diverse metadata related to the comparator's computations is reset as received messages indicate
    a change in usage.
    """
    def __init__(self, 
    spot_metadata_interface_class: type(SpotMetaDataInterface) = RedisSpotMetaDataInterface, 
    message_queue_interface_class: type(MessageQueueInterface) = RedisMessageQueueInterface,
    feature_extractor_class: type(FeatureExtractor) = SpinFeatureExtractor,
    past_features_queue_interface_class: type(PastFeatureDataQueuesInterface) = RedisFeatureDataQueuesInterface):

        self.subscriber_expert_system = rp.Subscriber(ROS_EXPERT_SYSTEM_UPDATE_TOPIC, String, self.callback)
        self.spots = dict()
        self.spot_metadata_interface = spot_metadata_interface_class()
        self.message_queue_interface = message_queue_interface_class()
        self.feature_extractor = feature_extractor_class()
        self.past_features_queue_interface = past_features_queue_interface_class()

    def callback(self, name_parameter_containing_exercises: str) -> NoReturn:
        spot_update_data = yaml.safe_load(name_parameter_containing_exercises.data)  # TODO: Fit this to API with tamer

        spot_queue_key, spot_past_queue_key, spot_info_key, spot_state_key, redis_spot_feature_progression_key, redis_spot_resampled_features_key = generate_redis_key_names(spot_update_data["stationID"])

        exercise_data = yaml.safe_load(rp.get_param(spot_update_data['parameterServerKey']))
        
        # TODO: In the future: Possibly use multiple recordings
        recording = self.feature_extractor.recording_to_ndarray(exercise_data['recording'])

        # TODO: Tamer must let experts specify the features of interest
        feature_of_interest_specification = self.feature_extractor.extract_feature_of_interest_specification_dictionary(exercise_data)

        reference_feature_trajectories = self.feature_extractor.extract_feature_trajectories_from_recordings([recording], feature_of_interest_specification)

        reference_feature_data = self.feature_extractor.extract_reference_feature_data_from_feature_trajectories(reference_feature_trajectories)

        beginning_pose = recording[0]
        exercise_data['beginning_state_dict'] = self.feature_extractor.extract_states(beginning_pose, reference_feature_data, feature_of_interest_specification)

        # The 'stages' entry is an artifact of an older interface to tamer
        del exercise_data['stages']
        
        # Set all entries that are needed by the comparators threads later on
        exercise_data['recording'] = recording
        exercise_data['feature_of_interest_specification'] = feature_of_interest_specification
        exercise_data['reference_feature_trajectories'] = reference_feature_trajectories
        exercise_data['reference_feature_data'] = reference_feature_data
        
        spot_info_dict = {'start_time': time.time_ns(), "exercise_data": exercise_data, 'repetitions': 0}

        if HIGH_VERBOSITY:
            rp.logerr("Updating info for: " + spot_info_key)

        self.spot_metadata_interface.delete(spot_state_key)
        self.past_features_queue_interface.delete(redis_spot_feature_progression_key)
        self.past_features_queue_interface.delete(redis_spot_resampled_features_key)
        self.message_queue_interface.delete(spot_queue_key)
        self.message_queue_interface.delete(spot_past_queue_key)

        self.spot_metadata_interface.set_spot_info_dict(spot_info_key, spot_info_dict)
        


if __name__ == '__main__':
    # initialize ros node
    rp.init_node('ComparingSystem_SpotMetaDataHandler', anonymous=False)

    spot_info_handler = SpotMetaDataHandler()

    rp.spin()
