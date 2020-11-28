catkin_make
source devel/setup.bash
sed -i 's/`//g' ./devel/include/pose_estimation/Persons.h
sed -i 's/`//g' ./devel/include/pose_estimation/Person.h
sed -i 's/`//g' ./devel/include/pose_estimation/Bodypart.h
sed -i 's/`//g' ./devel/include/pose_estimation/Pixel.h
