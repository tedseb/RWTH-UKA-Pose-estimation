docker stop trainerAI > /dev/null

docker run  -t --rm \
    --name trainerAI \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/trainerai \
    --device=/dev/video0 \
    registry.git.rwth-aachen.de/trainerai/core/trainerai-dev &2>1 > /dev/null &
