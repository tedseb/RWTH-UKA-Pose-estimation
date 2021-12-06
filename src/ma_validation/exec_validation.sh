#!/bin/bash

while getopts t: flag
do
    case "${flag}" in
        t) time=${OPTARG};;
        c) config_path=${OPTARG};;
    esac
done

echo "New validation running. killing after: $time seconds";

cd ~/trainerai-core/
timeout --signal=SIGINT $time roslaunch src/ma_validation/launch/validator.launch c:=${config_path}
