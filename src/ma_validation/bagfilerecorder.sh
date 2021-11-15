#!/bin/bash
cd ~/trainerai-core/data
timestamp=$(date +%s)
rosbag record -O ma_validation_dataset_$timestamp.bag /fused_skelleton /ma_validation_video_timing /ma_validation_set /station_usage /ma_validation_done