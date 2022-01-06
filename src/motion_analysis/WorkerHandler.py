#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the WorkerHandler Node.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

try:
    from motion_analysis.src.Worker import *
    from motion_analysis.src.InterCom import *
    from motion_analysis.src.DataUtils import *
    from motion_analysis.src.ROSAdapters import *
    from motion_analysis.src.algorithm.FeatureExtraction import *
    from motion_analysis.src.algorithm.AlgoUtils import *
    from motion_analysis.src.algorithm.GUI import *
    from backend.msg import StationUsage
except ImportError:
    from src.Worker import *
    from src.InterCom import *
    from src.DataUtils import *
    from src.ROSAdapters import *
    from src.algorithm.FeatureExtraction import *
    from src.algorithm.AlgoUtils import *
    from src.algorithm.GUI import *
    from backend.msg import StationUsage

from unittest import mock
import signal
import time
from PyQt5.QtCore import QThread
import yaml
from typing import NoReturn
import rospy as rp
from std_msgs.msg import String
import random
import argparse
import logy


class WorkerHandler(QThread):
    """Waits for updates on the usage of spots and communicates them through the a SpotMetaDataInterface.

    The WorkerHandler waits for updates of the usage of stations. It queries the HMI database for exercise data.
    This data includes recordings of experts and further data regarding the exercise.
    It uses a feature extractor to extract information that is found in the recording.
    These includes especially, but is not limited to, trajectories of features of interest. For example the trajectory of
    the angle of the left and right knee.

    The extracted features are set in a SpotMetaDataInterface and become avaiblable to the Comparator threads.
    Furthermore, diverse metadata related to the comparator's computations is reset as received messages indicate
    a change in usage.
    """
    def __init__(self,
    config,
    spot_metadata_interface_class: SpotMetaDataInterface = RedisSpotMetaDataInterface, 
    spot_queue_interface_class: SpotQueueInterface = RedisSpotQueueInterface,
    pose_definition_adapter_class: PoseDefinitionAdapter = MetrabsPoseDefinitionAdapter,
    features_interface_class: FeaturesInterface = RedisFeaturesInterface,
    enable_gui=False):
        super().__init__()
        self.config = config

        self.subscriber_expert_system = rp.Subscriber(self.config['ROS_STATION_USAGE_UPDATE_TOPIC'], StationUsage, self.callback)
        self.spots = dict()
        self.spot_metadata_interface = spot_metadata_interface_class()
        self.spot_queue_interface = spot_queue_interface_class(self.config)
        self.pose_definition_adapter = pose_definition_adapter_class()
        self.features_interface = features_interface_class()
        self.workers = {} # A dictionary, with spot IDs as keys

        mongo_client = pymongo.MongoClient(self.config['MONGO_DB_URI'])
        db = mongo_client.trainerai

        if pose_definition_adapter_class == MetrabsPoseDefinitionAdapter:
            self.exercises = db.exercises # TODO: Change this to metrabs_...
            self.recordings = db.recordings
            self.hmiExercises = db.hmiExercises
        elif pose_definition_adapter_class == SpinPoseDefinitionAdapter:
            self.exercises = db.spin_exercises
            self.recordings = db.spin_recordings
            self.hmiExercises = db.spin_hmiExercises
        else:
            raise NotImplementedError("There is not database connection setup in the WorkerHandler for this PoseDefinitionAdapter type.")
        if enable_gui:
            self.gui_handler = GUIHandler()
            self.gui = MotionAnaysisGUI()
            def kill_gui_hook():
                self.gui_handler.stop()
            rp.on_shutdown(kill_gui_hook)
            self.gui_handler.run(self.gui)
        else:
            self.gui = mock.MagicMock()

    @logy.catch_ros
    def callback(self, station_usage_data: Any) -> NoReturn:
        station_id = station_usage_data.stationID
        spot_queue_key, spot_past_queue_key, spot_info_key, spot_featuers_key = generate_redis_key_names(station_id, self.config)

        self.features_interface.delete(spot_featuers_key)
        self.spot_queue_interface.delete(station_id)

        logy.debug("Updating info for spot with key: " + str(spot_info_key))

        if station_usage_data.isActive:
            exercise_data = self.exercises.find_one({"name": station_usage_data.exerciseName})

            rp.logerr(exercise_data)

            if exercise_data is None:
                logy.error("Exercise with key " + str(station_usage_data.exerciseName) + " could not be found in database. Exercise has not been started.")
                return
            
            optimized_config = exercise_data.get("optimized_config", None)
            optimized_config_score = exercise_data.get("optimized_config_score", None)
            if optimized_config:
                config = optimized_config
                logy.info("Starting exercise with optimized config and score " + str(optimized_config_score))
            else:
                config = self.config
                logy.warning("Starting exercise without optimized config. Optimize for this exercise or import data to exercise DB!")

            # TODO: In the future: Possibly use multiple recordings
            recording, video_frame_idxs = self.pose_definition_adapter.recording_to_ndarray(exercise_data['recording'])
            recording = self.pose_definition_adapter.normalize_skelletons(recording)

            recordings = [recording]
            feature_of_interest_specification = extract_feature_of_interest_specification_dictionary(hmi_features=exercise_data['features'], pose_definition_adapter=self.pose_definition_adapter)

            # For now, we have the same pose definition adapter for all recordings
            reference_data = [(exercise_data["name"], r, self.pose_definition_adapter) for r in recordings]
            reference_recording_feature_collections = [ReferenceRecordingFeatureCollection(config, feature_hash, feature_specification, reference_data) for feature_hash, feature_specification in feature_of_interest_specification.items()]

            # Initialize features
            features_dict = {c.feature_hash: Feature(config, c) for c in reference_recording_feature_collections}
            self.features_interface.set(spot_featuers_key, features_dict)

            # Set all entries that are needed by the handler threads later on
            exercise_data['recordings'] = {fast_hash(r): r for r in recordings}
            exercise_data['video_frame_idxs'] = video_frame_idxs
            del exercise_data['features'] # We replace features with their specification dictionary, so we do not need them anymore here
            exercise_data['feature_of_interest_specification'] = feature_of_interest_specification
            exercise_data['reference_feature_collections'] = reference_recording_feature_collections

            spot_info_dict = {'start_time': time.time_ns(), "exercise_data": exercise_data, 'repetitions': 0, 'station_usage_hash': station_usage_data.stationUsageHash}

            self.spot_metadata_interface.set_spot_info_dict(spot_info_key, spot_info_dict)

            current_worker = self.workers.get(station_id, None)

            feature_hashes = [c.feature_hash for c in reference_recording_feature_collections]

            if current_worker:
                current_worker.running = False
                del self.workers[station_id]
                self.spot_metadata_interface.delete(spot_info_key)    
                self.gui.update_available_spots(spot_name=station_id, active=False)

            self.workers[station_id] = Worker(config, spot_key=station_id, gui=self.gui, pose_definition_adapter_class=self.pose_definition_adapter.__class__)
            self.gui.update_available_spots(spot_name=station_id, active=True, feature_hashes=feature_hashes)

            logy.info("New worker started for spot with key " + str(spot_info_key))
            logy.info("Current Workers: " + str(self.workers))
        else:
            current_worker = self.workers.get(station_id, None)

            logy.info("Current Workers: " + str(self.workers))

            if current_worker:
                current_worker.running = False
                del self.workers[station_id]
                self.spot_metadata_interface.delete(spot_info_key)
                self.gui.update_available_spots(spot_name=station_id, active=False)
                logy.warn("Worker stopped for spot with key " + str(spot_info_key))
            else:
                logy.error("Tried stopping non existent worker for spot with key " + str(spot_info_key))
            

if __name__ == '__main__':
    # initialize ros node
    # logy.basic_config(debug_level=logy.DEBUG, module_name="MA")
    rp.init_node('Motion_Analysis_WorkerHandler', anonymous=False)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="Verbose mode", action="store_true") # TODO: @sm This does not do anything now - is it even necessary?
    parser.add_argument("-a", "--ai", help="Name of AI to work with", type=str, default='metrabs')
    parser.add_argument("-c", "--configpath", help="Path to config file", type=str, default='/home/trainerai/trainerai-core/src/motion_analysis/config.yml')
    parser.add_argument("-g", "--enablegui", help="Whether to enable the gui", default=0, type=int)
    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]
    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()

    if args.ai == 'spin':
        pose_definition_adapter_class = SpinPoseDefinitionAdapter
    elif args.ai == 'metrabs':
        pose_definition_adapter_class = MetrabsPoseDefinitionAdapter
    else:
        logy.error("Could not find a suitable PoseDefinition Adapter for ai argument: <" + str(args.ai) + ">")

    with open(args.configpath, 'r') as infile:
        config = yaml.safe_load(infile)

    spot_info_handler = WorkerHandler(config, pose_definition_adapter_class=pose_definition_adapter_class, enable_gui=args.enablegui)

    rp.spin()
