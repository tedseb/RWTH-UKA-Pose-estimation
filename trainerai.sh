# This script downloads and build all files neccessary for the TrainerAI project
echo ">>  Setup ROS"
SCRIPT=`realpath $0`
SCRIPTPATH="/home/trainerai/trainerai-core"

# catkin_make builds the project with catkin in the folders src, build and evel
source /opt/ros/noetic/setup.bash
catkin_make
# catkin generates a setup file that sets a couple of environment variables
sh $SCRIPTPATH/devel/setup.sh
# TODO: @Shawan What does this line do? Is there a link you copied this from?
find ./devel -type f -name "*.js" -exec sed -i 's/`resolution`//g' {} \;

echo ">>  Start downloading large files from Google Drive"
if [[ ! -d './src/AI/spin/extra_data/body_module' ]]
then
    echo "./src/AI/spin/extra_data/body_module/' does not exist on your filesystem."
    echo ">>  Download extra data for body modules"
    python3 docker/scripts/largeFiles/SecureGoogleDrive.py
    sh script.sh
    rm script.sh
fi


