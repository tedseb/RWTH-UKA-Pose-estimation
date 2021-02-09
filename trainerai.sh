# This script downloads and build all files neccessary for the TrainerAI project

SCRIPT=`realpath $0`
SCRIPTPATH=`dirname $SCRIPT`

# catkin_make builds the project with catkin in the folders src, build and evel
catkin_make
# catkin generates a setup file that sets a couple of environment variables 
sh $SCRIPTPATH/devel/setup.sh
# TODO: @Shawan What does this line do? Is there a link you copied this from?
find ./devel -type f -name "*.js" -exec sed -i 's/`resolution`//g' {} \;


if [[ ! -d './src/AI/fastpose/parameters/' ]]
then
    echo "'./src/AI/fastpose/parameters/' does not exist on your filesystem."
    python3 docker/scripts/SecureGoogleDrive.py
    bash script.sh
    rm script.sh
fi


if [[ ! -d './src/AI/spin/extra_data/body_module' ]]
then 
    echo "./src/AI/spin/extra_data/body_module/' does not exist on your filesystem."
    echo ">>  Download extra data for body module"
    python3 docker/scripts/spin/SecureGoogleDrive.py
    sh script.sh
    rm script.sh
fi


