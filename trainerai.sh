catkin_make
source devel/setup.bash
find ./devel -type f -name "*.js" -exec sed -i 's/`resolution`//g' {} \;

if [[ ! -d './src/fastpose/parameters/' ]]
then
    echo "'./src/fastpose/parameters/' does not exist on your filesystem."
    python3 docker/scripts/SecureGoogleDrive.py
    bash script.sh
    rm script.sh
fi


if [[ ! -d './src/frankmocap/extra_data/body_module' ]]
then 
    echo "./src/frankmocap/extra_data/body_module/' does not exist on your filesystem."
    echo ">>  Download extra data for body module"
    python3 docker/scripts/frankmocap/SecureGoogleDrive.py
    bash script.sh
    rm script.sh
fi


