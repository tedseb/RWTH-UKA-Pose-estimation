# Build the base image with the general dependencies
docker build -t trainerai-base -f infrastructure/docker/base infrastructure/docker

# Build the dev image, which adds dev tools to the image
docker build -t trainerai-dev -f infrastructure/docker/dev .


# Build the release image, which builds the software and packs all the files in it
