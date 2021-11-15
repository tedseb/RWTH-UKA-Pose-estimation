#!/bin/bash
cd ~/trainerai-core/data
# This line is supposed to play back our rosbag file with 100x speed, waiting for the message queue interface to report back on the queue length of the motion analysis every second 
rosbag play dataset.bag # -r 100 --rate-control-topic="ma_validation_rosbag_rate_control" --rate-control-max-delay=1 