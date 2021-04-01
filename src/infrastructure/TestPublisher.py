#!/usr/bin/env python
# license removed for brevity
import rospy
import json
from std_msgs.msg import String
import yaml

def talker():
    pub = rospy.Publisher('test_pub', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)
    msg = json.dumps({'id' : 2, 'state' : True})
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass