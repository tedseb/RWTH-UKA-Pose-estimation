

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
We use the container registry from the RWTH Aachen gitlab instance. Pushing to it is time sensitive, but works from the RWTH network. You have to login to the registry by entering `docker login registry.git.rwth-aachen.de` and then provide your username (firstname.lastname most of the time) and the password you set (this is the password in gitlab, not the one you use with your TIM code to login!)
If you want to locally build an image to use, run the following command:.

	# You do not have to run this command, if you want to use the working container we have!
	bash docker/build_docker.sh

This might take a while. 

### Start a dev session
To start a dev session, you have to do 3 steps: Launch the container, go into it, and run the setup for your shell.
For the first step, run the

	bash docker/launch_docker.sh

command. This will kill the running docker container if it exists and then launch a new one. Note, that if you run this command while developing, all your open sessions will close. Use -g for gpu support, -w for webcam. For more information please check the launch file itself.

If you have the container running, you can use

	docker exec -it /trainerAI bash

to create a session in the dev container. The repository is mounted in the /trainerai folder, so all changes in there will be persistent on your host system. It is probably easiest to run the source control on your host system.
Use `source trainerai.sh` to build the project and set the environment variables. When building the project, large files will be downloaded from Google Drive, like neural networks. Therefore, a browser will open where you will be asked to log in to your Google account (social@trainerai.de). Then the files are downloaded automatically via OAuth2.0.
After building it, it can be run using launch files, so that we can use that to orchestrate the nodes. You can run `terminator -l layout` to launch 4 terminals, where roscore and htop are running by default. You are now ready to develop on the software.
To start all cameras use `roslaunch infrastructure all_cameras.launch`. To start the whole trainerAI system with all nodes `roslaunch launch/stack_3Dspin.launch`.
To visualize results, we want to use `rviz -d docker/scripts/default.rviz` and rqt. They are both available in the environment, I will add further information for this as we progress in the work.
## Docker
To allow for reproducible and shared environments, we use Docker to encapsulate the environment. At this point in time, we have 3 docker images: A **base** image which contains the requirements to run the software (i.e. all dependencies), a **dev** image, which adds development tools, such as an IDE, and a **release** image, which contains the packaged app.
I have pushed an up-to-date version 
Currently, the release system is not implemented, but will be added at a later date.
