#!/bin/bash
set -e
set -o pipefail

gpu_option=""
cpus_option=""
webcam_option=""
display=$DISPLAY
osx=false
mongo=false
env=false
backend=false

# functions
usage() {
    echo '---------------- help -------------------'
    echo '-h, --help                Show this help.'
    echo '-o, --osx		            Set display to "host.docker.internal:0" and your IP to the X access control list of the container'
    echo '-w, --linux-webcam        Enable webcam under linux.          (--device=/dev/video0)'
    echo '-d, --display DISPLAY     Speficy a display for X11.          (-e DISPLAY=$DISPLAY)'
    echo '-g, --gpu                 Hand over GPUs to Container.        (--gpus=all -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all)'
    echo '-c, --cpus decimal        Limit number of CPUs.               (--cpus decimal)'
    echo '-m, --mongo-db            Start Mongo-db.'
    echo '-e, --start-env           Start environment (terminator + rviz).'
    echo '-b, --start-backend       Start environment + backend.'
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
    -m|--mongo-db)
    mongo=true
    ;;
    -e|--start-env)
    env=true
    ;;
    -b|--start-backend)
    backend=true
    ;;
    -w|--webcam)
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
echo "Trying to stop the docker container. Killing after 10s..."
docker stop trainerAI > /dev/null || true
echo "done"
echo "Running new docker con tainer..."
# Run the image as a container with the specified option strings
docker run -it -d --rm \
        --name trainerAI \
        -e DISPLAY=$display \
	    -p 3000:3000 \
        --privileged=true \
        --net=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $(pwd):/home/trainerai/trainerai-core \
        -v /home/trainerai/trainerai-core/node_modules \
        -v /sys/fs/cgroup:/sys/fs/cgroup:ro \
        -v /var/run/dbus/:/var/run/dbus/:z \
        $webcam_option \
        $gpu_option \
        registry.git.rwth-aachen.de/trainerai/trainerai-core/trainerai-core-20 /sbin/init

if $mongo || $env || $backend; then
	if [ ! "docker container list -a | grep mongo-on-docker" ]; then
        docker run -d --name mongo-on-docker -p 27888:27017  -v expert_mongo:/data/db -e MONGO_INITDB_ROOT_USERNAME=mongoadmin -e MONGO_INITDB_ROOT_PASSWORD=secret mongo
    else
        docker start mongo-on-docker
    fi
fi

if $env; then
    docker exec -it /trainerAI docker/scripts/start_env.sh
fi

if $backend; then
    docker exec -it /trainerAI docker/scripts/start_backend.sh
fi

if $osx; then
	docker exec -it trainerAI sh -c "export DISPLAY=host.docker.internal:0 && xhost + $(ifconfig en0 | grep 'inet[ ]' | awk '{print$2}')"
fi

echo "done"

