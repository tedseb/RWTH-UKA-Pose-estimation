import rospy

DEBUG_MODE = False
VERBOSE_MODE = True
MAX_STATIONS_PER_PC = 8
MAX_STATIONS_PER_GPU = 4

def LOG_DEBUG(msg, debug = VERBOSE_MODE):
    if debug:
        rospy.loginfo(f"[SM] {msg}", logger_name="StationManager")

def LOG_INFO(msg):
    rospy.loginfo(f"[SM] {msg}", logger_name="StationManager")