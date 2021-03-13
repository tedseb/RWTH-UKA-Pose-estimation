# This message describes user correction information

# The following structure reflects the REST API between the Comparing System and the Smtartphone App 
# as specified under https://app.getguru.com/card/iGK7zMAT/Tech-Spec-REST-API-ComparingSystem-Smartphone-App
# For questions, refer to Artur


int16 user_id
string current_exercise_name
int16 repetitions
int16 seconds_since_last_exercise_start
int32 milliseconds_since_last_repetition
int8 repetition_score
int8 exercise_score
geometry_msgs/Point32 user_position

