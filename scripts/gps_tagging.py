#!/usr/bin/env python

import rospy
from sensor_msgs.msg import *

def callback(data):
    print 'lat'
    rospy.loginfo('I heard  %f', data.latitude)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/mavros/global_position/global', NavSatFix ,callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
