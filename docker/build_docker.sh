set -xe

#while [[ $# -gt 0 ]]
#do
#key="$1"

PUSH="false"
BUILD="true"

#case $key in
#    -p|--push)
#    EXTENSION=true
#    shift # past argument
#    ;;
#    -s|--skip-build)
#    SEARCHPATH=false
#    shift # past argument
#    ;;
#    *)    # unknown option
#    POSITIONAL+=("$1") # save it in an array for later
#    shift # past argument
#    ;;
#esac
#done
#set -- "${POSITIONAL[@]}" # restore positional parameters


if [ "$BUILD" = "true" ] ; then
    echo 'Building images'
    # Build the base image with the general dependencies
    docker build -t registry.git.rwth-aachen.de/trainerai/core/trainerai-base -f docker/base docker
    # Build the dev image, which adds dev tools to the image
    docker build -t registry.git.rwth-aachen.de/trainerai/core/trainerai-dev -f docker/dev .
fi

#if [ "$PUSH" = "true" ] ; then
#    echo 'Pushing built images to registry'
#    docker push registry.git.rwth-aachen.de/trainerai/core/trainerai-base
#    docker push registry.git.rwth-aachen.de/trainerai/core/trainerai-dev
#fi


# Build the release image, which builds the software and packs all the files in it
