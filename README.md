# multispectral_camera

#Intial Setup

For the Raspberry Pi 3 B+, we will be using a Xenial (16.04) Ubuntu image from Ubiquity Robotics. This image comes with ROS pre-installed and so is perfect for our on-board computer for the drone. 

The image can be downloaded [here] (https://downloads.ubiquityrobotics.com/pi.html).
We used the “2019-06-19-ubiquity-xenial-lxde” file

##Setting Up the Pi 3

Upon first boot, the Pi will resize it’s file system to fill the SD card, this may take a few moments.

The image comes with a Wifi access point which will come in use later but for now we will need to connect to the internet to grab updates and software packages. 

First, disconnect form the access point then connect to internet wifi/ethernet

    sudo apt-get update

    sudo apt-get upgrade

Since we are not running one of Ubiquity’s robots, run this command  to disable their startup scripts
    sudo systemctl disable magni-base


