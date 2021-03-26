#!/usr/bin/env python
# license removed for brevity
import rospy
import json
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('test_pub', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)
    msg = json.dump({"id" : 0, "station" : 3})
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass