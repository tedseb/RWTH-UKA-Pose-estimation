#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the WorkerHandler Node.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

try:
    from motion_analysis.src.Worker import *
    from motion_analysis.src.DataConfig import *
    from motion_analysis.src.InterCom import *
    from motion_analysis.src.DataUtils import *
    from motion_analysis.src.ROSAdapters import *
    from motion_analysis.src.algorithm.AlgoConfig import *
    from motion_analysis.src.algorithm.FeatureExtraction import *
    from motion_analysis.src.algorithm.AlgoUtils import *
    from motion_analysis.src.algorithm.GUI import *
except ImportError:
    from src.Worker import *
    from src.DataConfig import *
    from src.InterCom import *
    from src.DataUtils import *
    from src.ROSAdapters import *
    from src.algorithm.AlgoConfig import *
    from src.algorithm.FeatureExtraction import *
    from src.algorithm.AlgoUtils import *
    from src.algorithm.GUI import *

import time
from PyQt5.QtCore import QThread
import yaml
from typing import NoReturn
import rospy as rp
from std_msgs.msg import String

class WorkerHandler(QThread):
    """Waits for updates on the usage of spots and communicates them through the a SpotMetaDataInterface.
    
    The WorkerHandler waits for stringified YAMLs from the HMI. This data includes recordings of experts and further
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
    pose_definition_adapter_class: PoseDefinitionAdapter = SpinPoseDefinitionAdapter,
    features_interface_class: FeaturesInterface = RedisFeaturesInterface):

        super().__init__()

        self.subscriber_expert_system = rp.Subscriber(ROS_EXPERT_SYSTEM_UPDATE_TOPIC, String, self.callback)
        self.spots = dict()
        self.spot_metadata_interface = spot_metadata_interface_class()
        self.spot_queue_interface = spot_queue_interface_class()
        self.pose_definition_adapter = pose_definition_adapter_class()
        self.features_interface = features_interface_class()
        self.workers = {} # A dictionary, with spot IDs as keys
        
        self.gui_handler = GUIHandler()
        self.gui = MotionAnaysisGUI()
        self.gui_handler.run(self.gui)

    def callback(self, name_parameter_containing_exercises: str) -> NoReturn:
        spot_update_data = yaml.safe_load(name_parameter_containing_exercises.data)  # TODO: Fit this to API with tamer
        station_id = spot_update_data["stationID"]
        spot_queue_key, spot_past_queue_key, spot_info_key, spot_featuers_key = generate_redis_key_names(spot_key=station_id)

        self.features_interface.delete(spot_featuers_key)
        self.spot_queue_interface.delete(station_id)

        if HIGH_VERBOSITY:
                rp.logerr("Updating info for spot with key: " + spot_info_key)

        if spot_update_data["isActive"]:
            exercise_data = yaml.safe_load(rp.get_param(spot_update_data['parameterServerKey']))
            
            # TODO: In the future: Possibly use multiple recordings
            recording = self.pose_definition_adapter.recording_to_ndarray(exercise_data['recording'])

            # TODO: Tamer must let experts specify the features of interest
            recordings = [recording]
            feature_of_interest_specification = extract_feature_of_interest_specification_dictionary(exercise_data=exercise_data, pose_definition_adapter=self.pose_definition_adapter)

            # For now, we have the same pose definition adapter for all recordings
            recordings_and_adapters = [(r, self.pose_definition_adapter) for r in recordings]
            reference_feature_collections = [ReferenceFeatureCollection(feature_hash, feature_specification, recordings_and_adapters) for feature_hash, feature_specification in feature_of_interest_specification.items()]

            # Initialize features
            features_dict = {c.feature_hash: Feature(c) for c in reference_feature_collections}

            self.features_interface.set(spot_featuers_key, features_dict)

            # The 'stages' entry is an artifact of an older interface to tamer
            del exercise_data['stages']
            
            # Set all entries that are needed by the handler threads later on
            exercise_data['recordings'] = {hash(str(r)): r for r in recordings}
            exercise_data['feature_of_interest_specification'] = feature_of_interest_specification
            exercise_data['reference_feature_collections'] = reference_feature_collections
            
            spot_info_dict = {'start_time': time.time_ns(), "exercise_data": exercise_data, 'repetitions': 0}
            
            self.spot_metadata_interface.set_spot_info_dict(spot_info_key, spot_info_dict)

            current_worker = self.workers.get(station_id, None)
            if not current_worker:
                self.workers[station_id] = Worker(spot_key=station_id, gui=self.gui)

            feature_hashes = [c.feature_hash for c in reference_feature_collections]
            self.gui.update_available_spots(spot_name=station_id, active=True, feature_hashes=feature_hashes)
        else:
            current_worker = self.workers.get(station_id, None)
            if current_worker:
                current_worker.running = False
                del self.workers[station_id]
            self.gui.update_available_spots(spot_name=station_id, active=False)


if __name__ == '__main__':
    # initialize ros node
    rp.init_node('Motion_Analysis_WorkerHandler', anonymous=False)

    spot_info_handler = WorkerHandler()

    rp.spin()
