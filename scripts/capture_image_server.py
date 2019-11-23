#!/usr/bin/env python

from multispectral_camera.srv import CaptureImage,CaptureImageResponse
import rospy
from picamera import PiCamera
import RPi.GPIO as GPIO
from time import sleep
import paramiko
from subprocess import call

camera = PiCamera()
camera.start_preview()
i = 0

GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.OUT)
GPIO.output(21, GPIO.LOW)

ssh_client = paramiko.SSHClient()
ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
key = paramiko.RSAKey.from_private_key_file('/home/ubuntu/.ssh/id_rsa')
ssh_client.connect(hostname='raspberrypi.local',username='pi',password=None,pkey=key)


def handle_capture_image(req):
    global i
    GPIO.output(21, GPIO.HIGH)
    camera.capture("/home/ubuntu/images/rgb_%s.jpg" % i)
    print "Captured rgb_%s.jpg" % i
    i = i + 1
    GPIO.output(21,GPIO.LOW)
    return CaptureImageResponse(1)

def capture_image_server():
   
    rospy.init_node('capture_image_server')
    r = rospy.Service('capture_image', CaptureImage, handle_capture_image)
    stdin, stdout, stderr = ssh_client.exec_command('python3 pi_capture.py')
    print "Ready to capture images"
    rospy.spin()

if __name__ == "__main__":
   try:
       capture_image_server()
    
   except KeyboardInterrupt:
       print(": Interrupt received, stopping...")

   finally:
       print("\nCleaning up")
       rc = call('/home/ubuntu/catkin_ws/src/multispectral_camera/scripts/scp_images.sh')
       ssh_client.close()
       camera.stop_preview()
       GPIO.cleanup()
