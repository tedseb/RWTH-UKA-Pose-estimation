# This message describes user correction information

# The following structure reflects the REST API between the Comparing System and the Smtartphone App 
# as specified under https://app.getguru.com/card/iGK7zMAT/Tech-Spec-REST-API-ComparingSystem-Smartphone-App
# For questions, refer to Artur


user_id: int16
current_exercise_name: string
repetitions: int16
seconds_since_last_exercise_start: int16
milliseconds_since_last_repetition: int32
repetition_score: int8
exercise_score: int8
user_position: geometry_msgs/Point32 point 

