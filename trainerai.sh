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

