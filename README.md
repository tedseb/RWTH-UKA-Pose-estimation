# TrainerAI

## Getting Started
This guide will lead you through setting up the trainerAI stack.

### Setting up the host system
We assume you are running linux, preferably a ubuntu derivate (I am running Kubuntu 18.04). In regards to physical requirements, a nvidia GPU has to be installed in your computer. Software requirements are the proprietary Nvidia drivers installed, as well as CUDA 10.0 for the GPU, and nvidia-docker to run the containers.
For the drivers, please refer to your distributions wiki. For nvidia-docker, refer to [https://github.com/NVIDIA/nvidia-docker](https://github.com/NVIDIA/nvidia-docker).

### Cloning the repository
The main repository is the one you are currently in: [https://git.rwth-aachen.de/trainerai/core](https://git.rwth-aachen.de/trainerai/core)
When you clone this repository, make sure you recursively clone all submodules: 
```
git clone --recurse-submodules -j8 git@git.rwth-aachen.de:trainerai/core.git
cd core
```
This makes sure, that the repositories holding the code for the expert system, as well as the pose estimation are cloned as well.
We want to use dockerhub as repo for our images, but at this point in time, we have to build them locally before being able to use them. Run the following command to build them:

    bash infrastructure/docker/build_docker.sh
This might take a while. This only has to be done once (but running it multiple times, shouldn't make a difference).
 ### Start a dev session
 To start a dev session, you have to do 3 steps: Launch the container, go into it, and run the setup for your shell.
 For the first step, run the 
 
    bash infrastructure/docker/launch_docker.sh
command. This will kill the running docker container if it exists and then launch a new one. Note, that if you run this command while developing, all your open sessions will close. If you have the container running, you can use
 
    bash infrastructure/docker/launch_docker.sh
to create a session in the dev container. I guess, we will use Visual Studio Code for the development, which is included in the dev image, so you can launch it with the `code` command. There you can spawn additional shells to run multiple processes. You should run `source trainerai.sh` to make sure that the environment is setup correctly. This will in the future be done automatically. 
You are now ready to develop on the software. The repository is mounted in the /trainerai folder, so all changes in there will be persistent on your host system. It is probably easiest to run the source control on your host system.
## Docker
To allow for reproducible and shared environments, we use Docker to encapsulate the environment. At this point in time, we have 3 docker images: A **base** image which contains the requirements to run the software (i.e. all dependencies), a  **dev** image, which adds development tools, such as an IDE, and a **release** image, which contains the packaged app.
Currently, the release system is not implemented, but will be added at a later date.

