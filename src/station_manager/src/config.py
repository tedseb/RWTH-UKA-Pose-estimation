import rospy
import logy

DEBUG_MODE = False
VERBOSE_MODE = True
MAX_STATIONS_PER_PC = 8
MAX_STATIONS_PER_GPU = 4

logy.Logy().basic_config(debug_level=logy.INFO, module_name="SM")

def LOG_DEBUG(msg, debug = VERBOSE_MODE):
    if debug:
        #rospy.loginfo(f"[SM] {msg}", logger_name="StationManager")
        logy.debug(msg)

def LOG_INFO(msg):
    #rospy.loginfo(f"[SM] {msg}", logger_name="StationManager")
    logy.info(msg)

def LOG_ERROR(msg):
    #rospy.logerr(f"[SM] {msg}", logger_name="StationManager")
    logy.error(msg)