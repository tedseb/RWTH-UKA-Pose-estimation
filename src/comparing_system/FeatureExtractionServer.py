#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
This file contains the Feature Extraction Server.
It is written and maintained by artur.niederfahrenhorst@rwth-aachen.de.
"""

try:
    import comparing_system.src.FeatureExtraction as FeatureExtractionModule
except ModuleNotFoundError:
    import src.FeatureExtraction as FeatureExtractionModule

from flask import Flask
from flask import request

import inspect

functions = inspect.getmembers(FeatureExtractionModule, inspect.isfunction)

app = Flask(__name__)

@app.route('/', methods=['GET'])
def extract_features():
    if request.method == 'GET':
        method_string = request.args.get('function_name')

        try:
            function = functions[method_string]
        except KeyError:
            raise 
        var_names = function.__code__.co_varnames

        var_list = []
        for var_name in var_names:
            if var_name == "pose_array":
                var_list.append(FeatureExtractionModule.SpinPoseDefinitionAdapter.body_parts_to_ndarray(request.args.get(var_name)))
                continue
            var_list.append(request.args.get(var_name))

        # find out function and use pose for it
        return function(*var_names)
            