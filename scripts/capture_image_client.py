#!/usr/bin/env python

from multispectral_camera.srv import *
import rospy
import sys
import RPi.GPIO as GPIO

#prep pin 18 for rising edge trigger
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

def capture_image_client():
    rospy.wait_for_service('capture_image')
    try:
        capture_image = rospy.ServiceProxy('capture_image', CaptureImage)
        resp = capture_image(1);
        return resp.rtn
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%e

if __name__ == '__main__':
    try:
        while True:
            #wait for rising edge from cam trigger on pixhawk
            GPIO.wait_for_edge(18, GPIO.RISING)
            print 'high sig detected'
            #call client to execute capture
            capture_image_client()
            print 'ready to capture'
    except KeyboardInterrupt:
        print ('keyboard interrupt, closing')
        GPIO.cleanup()
    finally:
        GPIO.cleanup()
