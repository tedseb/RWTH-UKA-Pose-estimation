#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Feature Extraction Server.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

try:
    import motion_analysis.src.algorithm.FeatureExtraction as FeatureExtractionModule
    from motion_analysis.src.algorithm.FeatureExtraction import *
    from motion_analysis.src.algorithm.Features import *
    from motion_analysis.src.ROSAdapters import *
except ModuleNotFoundError:
    import src.algorithm.FeatureExtraction as FeatureExtractionModule
    from src.algorithm.FeatureExtraction import *
    from src.algorithm.Features import *
    from src.ROSAdapters import *

import rospy as rp
import inspect
import asyncio
import websockets
import json
import yaml

adapter = MetrabsPoseDefinitionAdapter()

functions = inspect.getmembers(FeatureExtractionModule, inspect.isfunction)
functions = {a: b for a, b in functions}

async def analyze(websocket, path):
    """ Extra
    """
    async for message in websocket:
        data = json.loads(message)

        var_list = []

        if data["extraction_method"] == "function":
            # TODO: Implement this
            method_string = data['function_name']

            function = functions[method_string]
            var_names = function.__code__.co_varnames
            for n in var_names:
                # Geth all vars here
                raise NotImplementedError("Singular function not supported right now")

            var_list.append(adapter.pose_to_nd_array(data["pose_array"]))
            return_value = yaml.dumps({"return_value": function(*var_names)})

        elif data["extraction_method"] == "reference_features":
            exercise_data = yaml.safe_load(data["exercise_data"])
            recording, _ = adapter.recording_to_ndarray(data["recording"])
            recordings_and_adapters = [(recording, adapter)]
            feature_of_interest_specification = FeatureExtractionModule.extract_feature_of_interest_specification_dictionary(hmi_features=exercise_data, pose_definition_adapter=adapter)
            reference_recording_feature_collections = [ReferenceRecordingFeatureCollection(feature_hash, feature_specification, recordings_and_adapters) for feature_hash, feature_specification in feature_of_interest_specification.items()]

            reference_features_dict = {c.feature_hash: c.asdict() for c in reference_recording_feature_collections}

            return_value = yaml.dumps(reference_features_dict)
        
        elif data["extraction_method"] == "motion_analysis":
            exercise_data = yaml.safe_load(data["exercise_data"])
            reference_recording, _ = adapter.recording_to_ndarray(data["reference_recording"])
            recording = adapter.recording_to_ndarray(data["recording"])
            recordings_and_adapters = [(reference_recording, adapter)]
            feature_of_interest_specification = FeatureExtractionModule.extract_feature_of_interest_specification_dictionary(hmi_features=exercise_data, pose_definition_adapter=adapter)
            reference_recording_feature_collections = [ReferenceRecordingFeatureCollection(feature_hash, feature_specification, recordings_and_adapters) for feature_hash, feature_specification in feature_of_interest_specification.items()]

            # Initialize features
            features_dict = {c.feature_hash: Feature(c).asdict() for c in reference_recording_feature_collections}

            for pose in recording:
                for f in features_dict.values():
                    f.update(pose, adapter)

            return_value = yaml.dumps(features_dict)

    await websocket.send(return_value)

start_server = websockets.serve(analyze, "0.0.0.0", 3002)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
