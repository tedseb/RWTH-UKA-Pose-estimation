set -e

PUSH="false"
BUILD="false"
BUILDUPDATE="false"
DEV=""
while [[ $# -gt 0 ]]
do
key="$1"
case $key in
    -p|--push)
    PUSH=true
    shift # past argument
    ;;
    -b|--build-orig)
    BUILD=true
    shift # past argument
    ;;
    -d|--dev)
    # path in the format: path/Dockfile
    BUILDUPDATE=true
    shift # past argument
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters


if [ "$BUILD" = "true" ] ; then
    echo 'Building images'
    # Build the base image with the general dependencies. 
    # "-t" will specify the name of the image and with "-f" you specify from where to  read a Dockerfile 
    docker build -t registry.git.rwth-aachen.de/trainerai/core/trainerai-base -f docker/base docker
    # Build the dev image, which adds dev tools to the image
    docker build -t registry.git.rwth-aachen.de/trainerai/core/trainerai-dev -f docker/dev .
fi

if [ "$BUILDUPDATE" = "true" ] ; then
    echo 'Building an update image'
    echo "Using the Dockfile with the path: " $1
    # Build the dev image, which adds dev tools to the image
    name=$(echo $1 | sed 's:.*/::')
    docker build -t registry.git.rwth-aachen.de/trainerai/core/trainerai-"${name}" -f $1 .
fi


if [ "$PUSH" = "true" ] ; then
	echo 'Pushing built images to registry'
	if [ "$BUILD" = "true" ] ; then
	    docker push registry.git.rwth-aachen.de/trainerai/core/trainerai-base
	    docker push registry.git.rwth-aachen.de/trainerai/core/trainerai-dev
	fi
	if [ "$BUILDUPDATE" = "true" ] ; then
	    docker push registry.git.rwth-aachen.de/trainerai/core/trainerai-"${name}"
	fi

fi




# Build the release image, which builds the software and packs all the files in it
