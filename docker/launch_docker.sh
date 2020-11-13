#!/bin/bash
set -e
HELP="false"
RUN_GPU="false"
RUN_CPU="false"
RUN_NO_VIDEO="false"
STOP="false"
while [[ $# -gt 0 ]]
do
key="$1"
case $key in
    -h|--help)
    HELP=true
    shift 
    ;;
    -b|--run)
    RUN_GPU=true
    shift 
    ;;
    -c|--run-cpu)
    RUN_CPU=true
    shift 
    ;;
    -n|--run-no-video)
    RUN_NO_VIDEO=true
    shift 
    ;;
    -s|--stop)
    STOP=true
    shift 
    ;;
    *)
    POSITIONAL+=("$1") 
    shift 
    ;;
esac
done
set -- "${POSITIONAL[@]}" 

if [ "$HELP" = "true" ] ; then
    echo '---------------- HELP -------------------'
    echo '-h, --help            Show this help.'
    echo '-r, --run             Launch docker. Default with GPU and video..'
    echo '-c, --run-cpu         Launch with only CPU support.'
    echo '-n, --run-no-video    Launch without video.'
    echo '-s, --stop            Stop the container.'
    echo 'Note: Different commands can be used combined.'
fi

if [ "$RUN_GPU" = "true" ] && [ "$RUN_CPU" = "false" ] && [ "$RUN_NO_VIDEO" = "false" ] ; then
    docker stop trainerAI &> /dev/null
    docker run -t -d --rm \
        --gpus=all \
        --name trainerAI \
        -e DISPLAY=$DISPLAY \
        -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $(pwd):/trainerai \
        -v /trainerai/node_modules \
        --device=/dev/video0 \
        registry.git.rwth-aachen.de/trainerai/core/trainerai-dev-update &> /dev/null &
    echo 'Launched Container trainerAI in GPU mode and video.'
fi

if [ "$RUN_CPU" = "true" ] && [ "$RUN_NO_VIDEO" = "true" ] ; then
    docker stop trainerAI &> /dev/null
    docker run -t -d --rm \
        --name trainerAI \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $(pwd):/trainerai \
        registry.git.rwth-aachen.de/trainerai/core/trainerai-dev-update &> /dev/null &
    echo 'Launched Container trainerAI in CPU mode and without video.'
fi

if [ "$RUN_CPU" = "true" ] && [ "$RUN_NO_VIDEO" = "false" ] ; then
    docker stop trainerAI &> /dev/null
    docker run -t -d --rm \
        --name trainerAI \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $(pwd):/trainerai \
        --device=/dev/video0 \
        registry.git.rwth-aachen.de/trainerai/core/trainerai-dev-update &> /dev/null &
    echo 'Launched Container trainerAI in CPU mode and video.'
fi

if [ "$RUN_CPU" = "false" ] && [ "$RUN_NO_VIDEO" = "true" ] ; then
    echo 'Not implemented.'
fi

if [ "$STOP" = "true" ] ; then
    docker stop trainerAI &> /dev/null
    echo 'Container stopped.'
fi



