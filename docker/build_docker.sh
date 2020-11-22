set -e

push="false"
build="false"
build_and_update="false"

while [ "${1+defined}" ]; do
key="$1"
shift  # Simple and safe loop over arguments: https://wiki.bash-hackers.org/scripting/posparams
case $key in
    -p|--push)
    push=true
    ;;
    -b|--build)
    build=true
    ;;
    -d|--dev)
    # Path in the format: path/Dockfile
    build_and_update=true
    ;;
    *)    # Positional parameter
    positional_parameters+=("$1") # save it in an array for later
    ;;
esac
done
set -- "${positional_parameters[@]}" # Restore positional parameters


if [ "$build" = "true" ] ; then
    echo 'building images'
    # build the base image with the general dependencies. 
    # "-t" specifies the name of the image and "-f"the location of the Dockerfile 
    docker build -t registry.git.rwth-aachen.de/trainerai/core/trainerai-base -f docker/base docker
    # build the dev image, which adds dev tools to the image
    docker build -t registry.git.rwth-aachen.de/trainerai/core/trainerai-dev -f docker/dev .
    # TODO: Include release image here
fi

if [ "$build_and_update" = "true" ] ; then
    echo 'building an update image'
    echo "Using Dockfile at " $1
    # build the dev image, which adds dev tools to the image
    name=$(echo $1 | sed 's:.*/::')
    docker build -t registry.git.rwth-aachen.de/trainerai/core/trainerai-"${name}" -f $1 .
fi


if [ "$push" = "true" ] ; then
	echo 'Pushing built images to registry'
	if [ "$build" = "true" ] ; then
	    docker push registry.git.rwth-aachen.de/trainerai/core/trainerai-base
	    docker push registry.git.rwth-aachen.de/trainerai/core/trainerai-dev
	fi
	if [ "$build_and_update" = "true" ] ; then
	    docker push registry.git.rwth-aachen.de/trainerai/core/trainerai-"${name}"
	fi

fi



# Example: bash docker/build_docker.sh --push --dev /home/shawan/PycharmProjects/core/docker/dev-update
# build the release image, which builds the software and packs all the files in it
