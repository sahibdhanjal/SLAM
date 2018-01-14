# Simultaneous Localization and Mapping - Differential Drive Robot
The repository consists of 2 components, one is a code base for the BeagleBone Green and the other is a code base for the Raspberry Pi 3

NOTE: The code for the GUI, drivers and hardware communications was already setup and given to us in a template code by our Lab instructor( Dr. Peter Gaskell ). Our reponsibility as a team was only to modify the template in order to achieve predetermined functionalities such as SLAM, robot locomotion and autonomous exploration.

## Raspberry Pi 3 (RPi3)
The board was connected to the Scanse Sweep Lidar and was responsible for handling all the processing required to sense the surrounding, generate a map, localize in it and eventually send motor commands via USB to the BeagleBone.
Monte Carlo Localisation and Occupancy Grid Based Mapping was used for the SLAM.

## BeagleBone Green (BBG)
The board was connected to the RPi3 via USB and was responsible for the motion of the robot. The motors were connected to the BBG and upon receiving commands from the RPi3, the controllers on this board were responsible for generating a suitable trajectory for the motion of the robot.

In order to fully understand what was trying to be achieved, it is advisable to go through the attached report.
