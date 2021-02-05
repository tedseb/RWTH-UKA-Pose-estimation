#!/bin/bash
set -e
set -o pipefail

gpu_option=""
cpus_option=""
webcam_option=""
display=$DISPLAY
osx=false

# functions
usage() {
    echo '---------------- help -------------------'
    echo '-h, --help                Show this help.'
    echo '-o, --osx		    Set display to "host.docker.internal:0" and your IP to the X access control list of the container'
    echo '-w, --linux-webcam        Enable webcam under linux.          (--device=/dev/video0)'
    echo '-d, --display DISPLAY     Speficy a display for X11.          (-e DISPLAY=$DISPLAY)'
    echo '-g, --gpu                 Hand over GPUs to Container.        (--gpus=all -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all)'
    echo '-c, --cpus decimal        Limit number of CPUs.               (--cpus decimal)'
    echo 'Note: Different options can be combined.'
}

while [ "${1+defined}" ]; do # Simple and safe loop over arguments: https://wiki.bash-hackers.org/scripting/posparams
key="$1"
shift 
case $key in
    -o|--osx)
    display=host.docker.internal:0
    osx=true
    ;;
    -h|--help)
    usage >&2;
    exit
    ;;
    -g| --gpus)
    gpu_option="--gpus=all -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all"
    ;;
    -c| --cpus)
    cpus_option="--cpus=${1}"
    shift
    ;;
    -w|--nebcam)
    webcam_option="--device=/dev/video2"
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
	-p 3000:3000 \
	--net=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $(pwd):/trainerai \
        -v /trainerai/node_modules \
        $webcam_option \
        $gpu_option \
        registry.git.rwth-aachen.de/trainerai/core/trainerai-dev-update > /dev/null

if $osx; then
	docker exec -it trainerAI sh -c "export DISPLAY=host.docker.internal:0 && xhost + $(ifconfig en0 | grep 'inet[ ]' | awk '{print$2}')"
fi

echo "done"
