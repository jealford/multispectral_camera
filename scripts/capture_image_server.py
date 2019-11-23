#!/usr/bin/env python

from multispectral_camera.srv import CaptureImage,CaptureImageResponse
import rospy
from picamera import PiCamera
import RPi.GPIO as GPIO
from time import sleep
import paramiko
from subprocess import call
from sensor_msgs.msg import *
import exif_gps

#prep camera and counter on pi3
camera = PiCamera()
camera.start_preview()
i = 0

#setup gpio for cam trigger for pi zero
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.OUT)
GPIO.output(21, GPIO.LOW)

#setup paramiko to execute capture script on pi zero
ssh_client = paramiko.SSHClient()
ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
key = paramiko.RSAKey.from_private_key_file('/home/ubuntu/.ssh/id_rsa')
ssh_client.connect(hostname='raspberrypi.local',username='pi',password=None,pkey=key)

#global variables to store gps coords
gps_lat = 0
gps_lng = 0
gps_alt = 0

def handle_capture_image(req):
    #image incrementer
    global i

    #set pin high for signal to pi zero
    GPIO.output(21, GPIO.HIGH)

    #capture image on pi3
    camera.capture("/home/ubuntu/images/rgb_%s.jpg" % i)
    file_name = "/home/ubuntu/images/rgb_%s.jpg" % i

    #tag image with gps data
    exif_gps.set_gps_location(file_name, gps_lat, gps_lng, gps_alt)
    
    print ("Captured rgb_%s.jpg \t %f lat. \t %f long. \t %f alt."
           %(i, gps_lat, gps_lng, gps_alt)) 

    #increment for next image
    i += 1
    
    #prep pin back low for next capture
    GPIO.output(21,GPIO.LOW)
    
    return CaptureImageResponse(1)

def callback(data):
    global gps_lat, gps_lng, gps_alt
    gps_lat = data.latitude
    gps_lng = data.longitude
    gps_alt = data.altitude
    
def capture_image_server():
    #intialize node
    rospy.init_node('capture_image_server')

    #subscriber for gps data
    rospy.Subscriber('/mavros/global_position/global', NavSatFix ,callback)

    #service for CaptureImage
    r = rospy.Service('capture_image', CaptureImage, handle_capture_image)

    #paramiko command to start script on pi zero
    stdin, stdout, stderr = ssh_client.exec_command('python3 pi_capture.py')

    print "Ready to capture images"

    #spin node until shutdown
    rospy.spin()

if __name__ == "__main__":
   try:
       capture_image_server()
    
   except KeyboardInterrupt:
       print(": Interrupt received, stopping...")

   finally:
       print("\nCleaning up")
       
       #helper script to scp images over from pi zero to pi 3
       rc = call('/home/ubuntu/catkin_ws/src/multispectral_camera/scripts/scp_images.sh')

       #clean up 
       ssh_client.close()
       camera.stop_preview()
       GPIO.cleanup()
