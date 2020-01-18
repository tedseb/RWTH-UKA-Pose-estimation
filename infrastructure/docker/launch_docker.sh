docker run -ti --rm \
    --name trainerAI \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/trainerai \
    --device /dev/video0 \
    trainerai