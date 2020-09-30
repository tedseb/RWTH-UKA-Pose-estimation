

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

command. This will kill the running docker container if it exists and then launch a new one. Note, that if you run this command while developing, all your open sessions will close. If you have the container running, you can use

	bash docker/into_docker.sh

to create a session in the dev container. You can use Visual Studio code to develop on the project. The latest version contains configuration files for the remote container extension. Install Visual Studio code and inside run `ext install ms-vscode-remote.vscode-remote-extensionpack` to install the extension. Note that the docker container has to be running for this to work.
You can also run `terminator -l layout` to launch 4 terminals, where roscore and htop are running by default.
You are now ready to develop on the software. The repository is mounted in the /trainerai folder, so all changes in there will be persistent on your host system. It is probably easiest to run the source control on your host system.
Use `source trainerai.sh` to build the project and set the environment variables. To only run the build process, you can also use `colcon build`.
An example is shown in the infrastructure folder on how to create a ros component. After building it, it can be run using the `rosrun infrastructure CameraNode.py` command. We can also use launch files, so that we can use that to orchestrate the nodes. To start all cameras use `roslaunch infrastructure all_cameras.launch`. To start the whole trainerAI system with all nodes `roslaunch launch/stack.launch`. Note: After the workspace has been built, it musst be add to the ROS environment you need to source the generated setup file under devel `. devel/setup.bash`
To visualize results, we want to use rviz and rqt. They are both available in the environment, I will add further information for this as we progress in the work.
## Docker
To allow for reproducible and shared environments, we use Docker to encapsulate the environment. At this point in time, we have 3 docker images: A **base** image which contains the requirements to run the software (i.e. all dependencies), a **dev** image, which adds development tools, such as an IDE, and a **release** image, which contains the packaged app.
I have pushed an up-to-date version 
Currently, the release system is not implemented, but will be added at a later date.
