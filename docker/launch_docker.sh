docker stop trainerAI &> /dev/null

docker run  -t -d --rm \
    --gpus=all \
    --name trainerAI \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/trainerai \
    -v /trainerai/node_modules \
    --device=/dev/video0 \
    registry.git.rwth-aachen.de/trainerai/core/trainerai-dev-update &> /dev/null &

