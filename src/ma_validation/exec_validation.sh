#!/bin/bash
echo "Starting new validation..."

while getopts c:t: flag
do
    case "${flag}" in
        t) 
            time=${OPTARG}
            echo "Maximum runtime will be $time"
        ;;
        c) 
            configpath=${OPTARG}
            echo "Reading config from $configpath"
        ;;
    esac
done

cd ~/trainerai-core/
timeout --signal=SIGINT $time roslaunch src/ma_validation/launch/validator.launch config:=$configpath
