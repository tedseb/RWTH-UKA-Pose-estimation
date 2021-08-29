#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Feature Extraction Server.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

try:
    import motion_analysis.src.algorithm.FeatureExtraction as FeatureExtractionModule
except ModuleNotFoundError:
    import src.algorithm.FeatureExtraction as FeatureExtractionModule

import rospy as rp
import inspect
import asyncio
import websockets
import json

adapter = FeatureExtractionModule.SpinPoseDefinitionAdapter()

functions = inspect.getmembers(FeatureExtractionModule, inspect.isfunction)
functions = {a: b for a, b in functions}

async def extract_features(websocket, path):
    async for message in websocket:
        data = json.loads(message)
        method_string = data.get('function_name')

        if method_string is None:
            json.dumps({})

        try:
            function = functions[method_string]
        except KeyError:
            raise

        var_names = function.__code__.co_varnames

        var_list = []
        for var_name in var_names:
            if var_name == "pose_array":
                var_list.append(adapter.pose_to_nd_array(data[var_name]))
            else:
                var_list.append(data[var_name])

    return_value = function(*var_names)
    rp.logerr(return_value)

    await websocket.send(return_value)

start_server = websockets.serve(extract_features, "localhost", 4444)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()