# Low Cost Drone with Multispectral Imaging System

This project is part of the BioSensing and Instrumentation Lab (BSAIL) at the University of Georgia. Our goal was to develop a low cost drone and imaging system for high throughput phenotyping for agricultural crops. Most high-end drones and sensor systems used in the research field can cost tens of thousands of dollars so we set out to develop a custom built option at a fraction of the price.

Our drone uses a Pixhawk 4 flight controller which is an open hardware project aimed at providing high-quality and low cost autopilot hardware. The flight controller runs the ArduPilot flight stack. Our first sensor system is a low cost multispectral camera using Raspberry Pi’s with RGB and NIR cameras.

Our goal with this documentation is to provide the parts and procedures of building this drone so that it can be applied by others on similar projects. As we continue this research we will expand this documentation as necessary. 

![The Drone](https://jakealford.com/github/images/drone.JPG)

# Parts List

[Parts](#Parts)

## Preparing the Pixhawk

A solid walkthrough of the basic configuration of the flight controller can be found [here](https://docs.px4.io/v1.9.0/en/config/). Follow the steps in this guide and then return here for subsequent setup.

For the onboard computer (Raspberry Pi 3B+, in our case), we need to set up the telemetry 2 port to handle MAVLink communication.  We do this by adjusting several parameters on the board. We can do this using QGroundControl that was installed in the aforementioned guide. Connect to the flight controller, either by USB or telemetry radio and open the Parameter’s tab in QGC.

Find the parameter `SERIAL2_PROTOCOL` and set it’s value to `MAVLink1`. Also set `SERIAL2_BAUD` to `921600`, this is the baud rate we will use for the UART communication.

![SerialParams](https://jakealford.com/github/images/serial2params.png)

## Setting up the Pi 3

For the Raspberry Pi 3 B+, we will be using a Xenial (16.04) Ubuntu image from Ubiquity Robotics. This image comes with ROS Kinetic pre-installed and so is perfect for our on-board computer for the drone.

The image can be downloaded [here](https://downloads.ubiquityrobotics.com/pi.html)
We used the “2019-06-19-ubiquity-xenial-lxde” file.

Use an image writing software like [Etcher](https://www.balena.io/etcher/) to write this image to your micro SD card for the Pi. A 16GB card minimum is recommended but even larger is prefered if planning to capture large amounts of high resolution images and/or video.

Upon first boot, the Pi will resize it’s file system to fill the SD card, this may take a few moments.

The username is `ubuntu` with password `ubuntu`.

The image comes with a Wifi access point which will come in use later but for now we will need to connect to the internet to grab updates and software packages. 

First, disconnect form the access point then connect to internet enabled wifi/ethernet and perform software updates.

    sudo apt-get update
    sudo apt-get upgrade

This will take some time.

Since we are not running one of Ubiquity’s robots, run this command  to disable their startup scripts.

    sudo systemctl disable magni-base

We then need to configure some settings on the Pi 3. Run the configuration dialog:

    sudo raspi-config 

Select `Interfacing Options` then `P1 Camera` and `Yes` to enable. Do the same for `P2 SSH`. We also need to configure several serial options on the config page. Select `P5 Serial`, select `No` for "Would you like a login shell to be accessible over serial?" and `Yes` for "Would you like the serial port hardware to be enabled?".

![Wiring to Pi](https://jakealford.com/github/images/cam_ssh.png)

For Raspberry Pi 3B+, the bluetooth module occupied uart serial port. To disable the bluetooth, add `dtoverlay=pi3-disable-bt` and `enable_uart=1` the end of **/boot/config.txt**. Also, edit the content of **/boot/cmdline.txt** to
`dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles`.

## Install MavROS

MavROS is a communication node for ROS with a proxy for a Ground Control Station. It allows us to send MAVLnk messages from the onboard computer to the flight controller. More information can be found [here](https://github.com/mavlink/mavros/blob/master/mavros/README.md).

To install on the Raspberry Pi 3, run the following commands:

    sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras

    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

To run geographiclib script, make sure to add execute permissions then run the script.

    chmod a+x install_geographiclib_datasets.sh
    sudo ./install_geographiclib_datasets.sh

Now is a good time to run a test. Hook up the Raspberry Pi 3 to the Pixhawk as shown below.

![Wiring to Pi](https://jakealford.com/github/images/wiring_pi_telem.png)

Power them both up and run the below code to in a terminal. (The first command sets the publishing rate to 10 Hrtz for all MavROS topics, which we found was needed for the publishing of topics to go through) 

    rosrun mavros mavsys rate --all 10
    roslaunch mavros apm.launch fcu_url:=/dev/ttyAMA0:921600

Open up a new terminal window and issue the below command. 

    rostopic echo /mavros/imu/mag

Should see IMU data published to the terminal as illustrated below.

![Pixhawk to Pi Communication Test](https://jakealford.com/github/images/imu_test.png)

## Setting Up the Pi Zero and Connect the two Pi's

The second Raspberry Pi we will use is a Raspberry Pi Zero. Ours runs the lightweight version of Raspbian Buster. That image can be downloaded [here](https://www.raspberrypi.org/downloads/raspbian/).

After downloading, use image writing software like [Etcher](https://www.balena.io/etcher/) to  write this image to your micro SD card and when finished **DO NOT** remove from your computer just yet. Continue below.  

To interface the Rasberry Pi 3 and Pi Zero, we chose to go with setting up the Zero as an 'ethernet gadget'. This allows us to network the two microcontrollers together so that we can SSH into the Pi Zero from the Pi 3 without using a wireless connection or ethernet port. We can also SCP images collected from the Pi Zero camera over to the Pi 3 for easier retrevial of data post flight.

To begin the connections we must do some configuration on the SD card with the Raspbian OS on the Pi Zero first. On the root directory (i.e. /boot) open the file `config.txt` with your prefered text editor using sudo. Scroll down to the bottom of this file and add `dtoverlay=dwc2` to a new line. Save and exit. 

![Eidt config.txt](https://jakealford.com/github/images/configtext.png)

Next open `cmdline.txt` and scroll across to find the command `rootwait`. After `rootwait` add `modules-load=dwc2,g_ether` being careful not to add and extra spaces or newline characters.  

![Eidt config.txt](https://jakealford.com/github/images/cmdlinetext.png)

Lastly, we need to enable SSH so that we can communicate with the Pi Zero once it's connected to the Pi3. We can do this by placing a file name `ssh`, without an extension, onto the boot partition.

Now we can connect the Pi's and test the connection. You will need a Type B Micro to type A USB cable (Male to Male) that has data capabilities (ie not just voltage and ground). Note that the Micro USB plug goes into the port labeled `USB` on the Pi zero (not the `PWR IN`). This will provide both the data connection and power to the Pi Zero.

Power up the Pi 3 and the Pi Zero should power itself up. After the Pi 3 has booted up, open a terminal window and test the connection.

    ping raspberrypi.local

If successfull, next SSH into the Pi Zero.

    ssh pi@raspberrypi.local

The default password is `raspberry`.

Lastly, we need to enable the camera interface similair to how we did with the Pi 3.

    sudo raspi-config 

Select `Interfacing Options` then `P1 Camera` and `Yes` to enable.

### Setting up Camera Trigger on Pi Zero

We are using a GPIO interrupt to trigger the camera on the Pi Zero. When the image capture command is received by the Pi 3, our main onboard computer running ROS, it sets a GPIO pin high that is connected to a GPIO pin on the Pi Zero that has code running to catch this signal and trigger its camera.

Our code uses the GPIO21 on both the Pi 3 and Pi Zero for the interrupt but it can be tailored to any pin if need be, just update the code. Also the two Pi's need to share a common ground. A wiring diagram illustrates our setup below.

## Pixhawk Camera Trigger

Now it is time to configure the Pixhawk's camera trigger. Power up the flight controller and connect it to QGroundControl through either USB or telemetry. Open the Parameters tab and in the side pane select `CAM`. Set the following parameters: 
* `CAM_RELAY_ON` : `High`
* `CAM_TRIGG_TYPE` : `Relay`
* `CAM_DURATION` : `5 ds`

Then in the side pane select `RELAY` and the following parameters:
* `RELAY_DEFAULT` : `Off`
* `RELAY_PIN` : `Pixhawk AUXOUT5`

The Pixhawk camera trigger is now on AUXOUT5 which we then connect to another GPIO Pin on the Raspberry Pi 3 that is configured with an interupt. The wiring is illustrated below.

## Capturing an Image

We now have everything set up to capture images using the Pixhawk and on board computer. Connect the camera's to each Pi: the RGB to the Pi 3, and NIR to the Pi Zero. Boot up both the flight controller and Pi and open a terminal to run these commands:

    rosrun mavros mavsys rate --all 10
    roslaunch mavros apm.launch fcu_url:=/dev/ttyAMA0:921600

Open a new terminal tab and run the image capture server that starts the ROS service for the camera. 

    rosrun multispectral_camera capture_image_server.py

To capture an image call the service in a new terminal.

   rosservice call /capture_image 1

When the ROS capture_image_server is terminated (i.e. Ctrl-c) the images from the Pi Zero will be automatically SCP over to the Pi 3 and are stored in `~/images`.

## Data Processing

Each camera triggering will capture an RGB and NIR photo. We are currently using these two images to compute NDVI, Normalized Difference Vegetation Index, which is a scale that estimates plant chlorophyll content. An example we captured with this system is below.



## Parts

|                                        | QTY | Price   | Total Price |                                                                                                                                                                                                                                                                                                                                    | Vendor    |
|----------------------------------------|-----|---------|-------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| **Drone**                                  |     |         |             |                                                                                                                                                                                                                                                                                                                                    |           |
| PX4 Flight Controller Combo            | 1   | $219.99 | $219.99     | [Link](https://www.getfpv.com/pixhawk-4-autopilot-and-neo-m8n-gps-pm07-combo.html)                                                                                                                                                                                                                                                 | GetFPV    |
| Telemetry Radio Set                    | 1   | $39.00  | $39.00      | [Link](https://www.getfpv.com/holybro-100mw-transceiver-telemetry-radio-set-v3-915mhz.html)                                                                                                                                                                                                                                        | GetFPV    |
| FrSky Taranis QX7 TX                   | 1   | $107.99 | $107.99     | [Link](https://www.getfpv.com/frsky-taranis-q-x7-2-4ghz-16ch-transmitter-white.html)                                                                                                                                                                                                                                               | GetFPV    |
| Lumenier 2s LiPo TX Battery            | 1   | $18.99  | $18.99      | [Link](https://www.getfpv.com/lumenier-2100mah-2s-life-q-x7-radio-transmitter-battery.html)                                                                                                                                                                                                                                        | GetFPV    |
| Lumenier 30A BLHeli ESC                | 4   | $12.99  | $51.96      | [Link](https://www.getfpv.com/lumenier-30a-blheli-s-esc-opto-2-4s.html)                                                                                                                                                                                                                                                            | GetFPV    |
| Lumenier 5.2Ah 4s LiPo                 | 1   | $59.99  | $59.99      | [Link](https://www.getfpv.com/lumenier-5200mah-4s-35c-lipo-battery.html)                                                                                                                                                                                                                                                           | GetFPV    |
| HQ Prop 1045 CW (2 pack)               | 2   | $5.80   | $11.60      | [Link](https://www.getfpv.com/hqprop-10x4-5-cw-propeller-slow-flyer-2-blade-2-pack.html)                                                                                                                                                                                                                                           | GetFPV    |
| HQ Prop 1045 CCW (2 pack)              | 2   | $5.82   | $11.64      | [Link](https://www.getfpv.com/hqprop-10x4-5-ccw-propeller-multi-rotor-2-blade-2-pack.html)                                                                                                                                                                                                                                         | GetFPV    |
| XT60 (5 pairs)                         | 1   | $4.99   | $4.99       | [Link](https://www.getfpv.com/xt60-power-connectors-5-pair.html)                                                                                                                                                                                                                                                                   | GetFPV    |
| 18AWG Silicone Wire (2 Red, 2 Black)   | 4   | $1.29   | $5.16       | [Link](https://www.getfpv.com/silicone-wire-18awg-1mtr.html)                                                                                                                                                                                                                                                                       | GetFPV    |
| 16AWG Silicone Wire (2 Red, 2 Black)   | 4   | $1.59   | $6.36       | [Link](https://www.getfpv.com/silicone-wire-14awg-1mtr.html)                                                                                                                                                                                                                                                                       | GetFPV    |
| 3.5mm Gold Bullet Connectors (12 Pair) | 3   | $2.99   | $8.97       | [Link](https://www.getfpv.com/3-5mm-gold-bullet-connectors-12-pair.html)                                                                                                                                                                                                                                                           | GetFPV    |
| GemFam 9047 Props (4CCW, 4CW)          | 1   | $10.99  | $10.99      | [Link](https://www.amazon.com/Genuine-Propellers-Phantom-RAYCorp-RAYCorp/dp/B01MZ1ICOK/ref=sr_1_1?keywords=9047+prop&qid=1571186570&s=toys-and-games&sr=1-1)                                                                                                                                                                       | Amazon    |
| FrySky X8R RX                          | 1   | $39.98  | $39.98      | [Link](https://www.amazon.com/FrSky-Taranis-Compatible-Receiver-8-Channel/dp/B00RCAHHFM/ref=asc_df_B00RCAHHFM/?tag=hyprod-20&linkCode=df0&hvadid=242048352875&hvpos=1o1&hvnetw=g&hvrand=15210555723016150379&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=1015253&hvtargid=aud-829758849484:pla-588530700237&psc=1) | Amazon    |
| Readytosky S500 Frame                  | 1   | $46.99  | $46.99      | [Link](https://www.amazon.com/Readytosky-Quadcopter-Stretch-Version-Landing/dp/B01N0AX1MZ/ref=sr_1_9?keywords=drone+frame&qid=1569418692&s=gateway&sr=8-9)                                                                                                                                                                         | Amazon    |
| FliteTest 2212B 1050kV Motor           | 4   | $24.99  | $99.96      | [Link](https://store.flitetest.com/flite-test-radial-2212b-1050kv-brushless-motor-flt-3034/p846360)                                                                                                                                                                                                                                | FliteTest |
|                                        |     |         |             |                                                                                                                                                                                                                                                                                                                            |           |
| **Multispectral Camera**                   |     |         |             |                                                                                                                                                                                                                                                                                                                            |           |
| Raspberry Pi 3B+                       | 1   | $49.99  | $49.99      | [Link](https://www.amazon.com/ELEMENT-Element14-Raspberry-Pi-Motherboard/dp/B07BDR5PDW/ref=sr_1_4?keywords=Raspberry+Pi+3b%2B&qid=1570395844&s=electronics&sr=1-4)                                                                                                                                                                 | Amazon    |
| Raspberry Pi Zero W                    | 1   | $10.00  | $10.00      | [Link](https://www.adafruit.com/product/3400)                                                                                                                                                                                                                                                                                      | Adafruit  |
| RPi Zero camera cable                  | 1   | $5.95   | $5.95       | [Link](https://www.adafruit.com/product/3157)                                                                                                                                                                                                                                                                                      | Adafruit  |
| Raspberry Pi NoIR Camera               | 2   | $29.95  | $59.90      | [Link](https://www.adafruit.com/product/3100)                                                                                                                                                                                                                                                                                      | Adafruit  |
| Raspberry Pi Camera v2 (RGB)           | 1   | $29.95  | $29.95      | [Link](https://www.adafruit.com/product/3099)                                                                                                                                                                                                                                                                                      | Adafruit  |
| SanDisk 32GB MicroSD (2 pack)          | 1   | $13.15  | $13.15      | [Link](https://www.amazon.com/SanDisk-MicroSD-Ultra-UHS-1-Memory/dp/B00CNYV942/ref=sr_1_8?keywords=micro+sd+card+2+pack&qid=1570893964&sr=8-8%5C)                                                                                                                                                                                  | Amazon    |
| 0.5Ft Micro USB Cable                  | 1   | $4.99   | $4.99       | [Link](https://www.amazon.com/CableCreation-Charger-Compatible-Chromecast-Android/dp/B013G4EAEI/ref=sr_1_4?crid=10FF0DAIA31XN&keywords=short+usb+a+to+micro+usb+cable&qid=1570395913&s=electronics&sprefix=short+usb+a+to+mic%2Celectronics%2C169&sr=1-4)                                                                          | Amazon    |
| 4ft Micro USB Cable                    | 1   | $5.99   | $5.99       | [Link](https://www.amazon.com/CableCreation-Charger-Compatible-Chromecast-Android/dp/B013G4EDKY/ref=sr_1_4?crid=10FF0DAIA31XN&keywords=short%2Busb%2Ba%2Bto%2Bmicro%2Busb%2Bcable&qid=1570395913&s=electronics&sprefix=short%2Busb%2Ba%2Bto%2Bmic%2Celectronics%2C169&sr=1-4&th=1)                                                 | Amazon    |
| Raspberry Pi 3 B+ Heatsink Set         | 1   | $4.99   | $4.99       | [Link](https://www.amazon.com/LoveRPi-Performance-Heatsink-Set-Raspberry/dp/B018BGRDVS)                                                                                                                                                                                                                                            | Amazon    |
| Pixhawk Cable Set                      | 1   | $8.44   | $8.44       | [Link](https://www.amazon.com/dp/B01N4IRVQI/?coliid=I3UP89FP3QOA1B&colid=305F9Y448FVK8&psc=1&ref_=lv_ov_lig_dp_it)                                                                                                                                                                                                                 | Amazon    |
