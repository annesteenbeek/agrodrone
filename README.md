# Agrodrone
This repository contains the files for an automated spraying drone

## Overview
The software components consists of 4 main nodes
- agrodrone
- mavros plugins
- tank_level
- sprayer_controller

### agrodrone node
The agrodrone node is the main control node, it manages the flight missions and contains all the logic for managing the missions.

### mavros plugins
The agrodrone mavros plugins take care of the communication. They receive and send mavlink messages between ROS and the GCS. 

### tank_level
The tank_level node reads the pressure sensor data from the ADC chip and publishes this information in ros so it can be used by
the agrodrone node and also sent via mavlink to the GCS by using a mavros plugin. Additional information about connecting this can be found in the 
tank_level folder readme.

### sprayer_controller
The sprayer controller is a node which controls the PWM motors for the sprayers. 

## Install guide 
### Requirements
To be able to run this system, the following requirements must be met.

- Ubuntu 16.04 installed
- ROS Kinetic
- WiringPi library installed
- Mavros installed

### ROS install
First, you should have ubuntu 16.04 running on a board, the current setup was tested using an Odroid-C2, but it should also work on a raspberry pi 3.
The information on how to do this can be found on the website of each board, but should not be more complicated then flashing the image on an SD card.

The installation of ROS kinetic should also be pretty straight forward. The guide for this can be found here:  [ROS ARM INSTALL GUIDE](http://wiki.ros.org/action/show/kinetic/Installation/Ubuntu?action=show&redirect=kinetic%2FInstallation%2FUbuntuARM)
Once ROS is confirmed running, the first package to be installed should be Mavros, the instructions for this can be found on the mavros github page: [here](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)
However, you can skip the very last step for now (the building of mavros using catkin build).

You can then clone this repository into the `~/catkin_ws/src/` directory. Note, before you can do this, you need to add a deploy key from this repository to the device, the 
information on how to do this can be found on the github tutorial page (search for adding deploy key). Also make sure this repo is in the most up to date branch,
currently this is done using `git checkout kinetic-dev` while in the agrodrone main repo. Also make sure you clone using git instead of https so your deploy key is used.

Then do the following steps:
```
# Copy the mavlink message set to the mavlink folder
cd ~/catkin_ws/src/agrodrone/message_definitions/
cp agrodrone.xml ~/catkin_ws/src/mavlink/message_definitions/v1.0/
```
Now add the following line to the ardupilotmega.xml file below the line where `common.xml` is included.
```
<include>agrodrone.xml</include>
```
This way, mavros is able to use our new messages.

Once this is done, and all the dependencies are met, you can build the ros packages.

```
# Build all the catkin packages
cd ~/catkin_ws
catkin clean # This makes sure that the mavlink library is rebuild with our new messages
catkin build -j2 # This might take a while
source ~/.bashrc # make sure the newly created packages are in your path
```
Note that the `-j2` component is added since using all 4 cores on the odroid causes it to crash, and using more then 4 cores might strange cause build errors due to too many parallel builds interferring.

You can now launch the agrodrone package using `roslaunch agrodrone agrodrone.launch`

See the individual node pages to read up on the wiring needed for each component.

### Ardupilot setup

### GCS setup

### Boot at launch
