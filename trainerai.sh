catkin_make
source devel/setup.bash
sed -i 's/`//g' ./devel/include/pose_estimation/Persons.h
sed -i 's/`//g' ./devel/include/pose_estimation/Person.h
sed -i 's/`//g' ./devel/include/pose_estimation/Bodypart.h
sed -i 's/`//g' ./devel/include/pose_estimation/Pixel.h

if [[ ! -d './src/fastpose/parameters/' ]]
then
    echo "'./src/fastpose/parameters/' does not exist on your filesystem."
    python3 docker/scripts/SecureGoogleDrive.py
    bash script.sh
    rm script.sh
fi

