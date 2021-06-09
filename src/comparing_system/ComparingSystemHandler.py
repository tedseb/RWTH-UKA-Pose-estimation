#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Spot MetaDataHandler Node.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

try:
    from comparing_system.src.Comparator import Comparator
    from comparing_system.src.config import *
    from comparing_system.src.FeatureExtraction import *
    from comparing_system.src.InterCom import *
    from comparing_system.src.Util import *
except ImportError:
    from src.Comparator import Comparator
    from src.config import *
    from src.FeatureExtraction import *
    from src.InterCom import *
    from src.Util import *

import time
import rospy as rp
import yaml
from std_msgs.msg import String
from typing import NoReturn


class ComparingSystemHandler():
    """Waits for updates on the usage of spots and communicates them through the a SpotMetaDataInterface.
    
    The ComparingSystemHandler waits for stringified YAMLs from the HMI. This data includes recordings of experts and further
    data regarding the exercise. It uses a feature extractor to extract information that is found in the recording.
    These includes expecially, but is not limited to, trajectories of features of interest. For example the trajectory of
    the angle of the left and right knee.

    The extracted features are set in a SpotMetaDataInterface and become avaiblable to the Comparator threads.
    Furthermore, diverse metadata related to the comparator's computations is reset as received messages indicate
    a change in usage.
    """
    def __init__(self, 
    spot_metadata_interface_class: SpotMetaDataInterface = RedisSpotMetaDataInterface, 
    spot_queue_interface_class: SpotQueueInterface = RedisSpotQueueInterface,
    feature_extractor_class: FeatureExtractor = SpinFeatureExtractor,
    past_features_queue_interface_class: PastFeatureDataQueuesInterface = RedisFeatureDataQueuesInterface):

        self.subscriber_expert_system = rp.Subscriber(ROS_EXPERT_SYSTEM_UPDATE_TOPIC, String, self.callback)
        self.spots = dict()
        self.spot_metadata_interface = spot_metadata_interface_class()
        self.spot_queue_interface = spot_queue_interface_class()
        self.feature_extractor = feature_extractor_class()
        self.past_features_queue_interface = past_features_queue_interface_class()
        self.comparators = {} # A dictionary, with spot IDs as keys

    def callback(self, name_parameter_containing_exercises: str) -> NoReturn:
        spot_update_data = yaml.safe_load(name_parameter_containing_exercises.data)  # TODO: Fit this to API with tamer
        station_id = spot_update_data["stationID"]
        spot_queue_key, spot_past_queue_key, spot_info_key, spot_state_key, redis_spot_feature_progression_key, redis_spot_resampled_features_key = generate_redis_key_names(station_id)

        self.spot_metadata_interface.delete(spot_state_key)
        self.past_features_queue_interface.delete(redis_spot_feature_progression_key)
        self.past_features_queue_interface.delete(redis_spot_resampled_features_key)
        self.spot_queue_interface.delete(station_id)

        if HIGH_VERBOSITY:
                rp.logerr("Updating info for spot with key: " + spot_info_key)

        if spot_update_data["isActive"]:
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
            
            self.spot_metadata_interface.set_spot_info_dict(spot_info_key, spot_info_dict)

            current_comparator = self.comparators.get(station_id, None)
            if not current_comparator:
                self.comparators[station_id] = Comparator(spot_key=station_id)
        else:
            current_comparator = self.comparators.get(station_id, None)
            if current_comparator:
                current_comparator.running = False
                del self.comparators[station_id]


if __name__ == '__main__':
    # initialize ros node
    rp.init_node('ComparingSystem_ComparingSystemHandler', anonymous=False)

    spot_info_handler = ComparingSystemHandler()

    rp.spin()
