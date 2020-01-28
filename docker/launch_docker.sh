docker stop trainerAI > /dev/null

docker run  -t --rm \
    --runtime=nvidia \
    --name trainerAI \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/trainerai \
    --device=/dev/video0 \
    registry.git.rwth-aachen.de/trainerai/core/trainerai-dev &2>1 > /dev/null &
