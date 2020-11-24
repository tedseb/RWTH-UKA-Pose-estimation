#!/bin/bash
set -e
set -o pipefail

gpu_option="--gpus=all -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all"
cpus_option=""
webcam_option="--device=/dev/video0"
display=$DISPLAY

# functions
usage() {
    echo '---------------- help -------------------'
    echo '-h, --help                Show this help.'
    echo '-w, --linux-webcam        Enable Linux Webcam.                (--device=/dev/video0)'
    echo '-d, --display DISPLAY     Speficy a display for X11.          (-e DISPLAY=$DISPLAY)'
    echo '-g, --nogpu              Hand over GPUs to Container.         (omit --gpus=all -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all)'
    echo '-c, --cpus decimal        Limit number of CPUs.               (--cpus decimal)'
    echo '-w, --nowebcam           Launch without webcam video input.   (omit --device=/dev/video0)'
    echo 'Note: Different options can be combined.'
}

while [ "${1+defined}" ]; do # Simple and safe loop over arguments: https://wiki.bash-hackers.org/scripting/posparams
key="$1"
shift 
case $key in
    -h|--help)
    usage >&2;
    exit
    ;;
    -g| --nogpu)
    gpu_option=""
    ;;
    -c| --cpus)
    cpus_option="--cpus=${1}"
    shift
    ;;
    -w|--nowebcam)
    webcam_option=""
    ;;
    -d|--display)
    display=${1}
    shift
    ;;
    *)
    echo "Unknown option or parameter $key"
    usage >&2;
    echo "Exiting..."
    exit
    ;;
esac
done

# Stop any running container named 'trainerAI'
echo "Gracefully stopping docker container. Killing after 10s..."
docker stop trainerAI > /dev/null || true
echo "done"
echo "Running new docker container..."
# Run the image as a container with the specified option strings
docker run -it -d --rm \
        --name trainerAI \
        -e DISPLAY=$display \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $(pwd):/trainerai \
        -v /trainerai/node_modules \
        $webcam_option \
        $gpu_option \
        registry.git.rwth-aachen.de/trainerai/core/trainerai-dev-update > /dev/null
echo "done"