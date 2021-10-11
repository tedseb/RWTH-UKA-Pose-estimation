# This script downloads and build all files neccessary for the TrainerAI project
echo ">>  Setup ROS"
SCRIPT=`realpath $0`
SCRIPTPATH=`dirname $SCRIPT`

# catkin_make builds the project with catkin in the folders src, build and evel
catkin_make
# catkin generates a setup file that sets a couple of environment variables 
sh $SCRIPTPATH/devel/setup.sh
# TODO: @Shawan What does this line do? Is there a link you copied this from?
find ./devel -type f -name "*.js" -exec sed -i 's/`resolution`//g' {} \;

if [[ ! -d './src/AI/spin/extra_data/body_module' ]]
then 
    python3 docker/scripts/largeFiles/SecureGoogleDrive.py
    sh script.sh
    rm script.sh
fi

