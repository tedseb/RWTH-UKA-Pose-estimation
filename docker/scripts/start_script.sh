#!/bin/bash
redis-server --port 5678
LD_LIBRARY_PATH=/usr/local/cuda-11.2/lib64:$LD_LIBRARY_PATH