#!/bin/bash

# Permissions
chgrp gpio  /sys/class/gpio/export /sys/class/gpio/unexport
chmod 775 /sys/class/gpio/export /sys/class/gpio/unexport

# setup laser pin
echo 172 > /sys/class/gpio/export
LASER=/sys/class/gpio/gpio172
chgrp -HR gpio $LASER
chmod -R  775  $LASER
echo "out" > $LASER/direction
echo "0" > $LASER/active_low
echo "0" > $LASER/value

# setup user button pin
echo 174 > /sys/class/gpio/export
USR_BTN=/sys/class/gpio/gpio174
chgrp -HR gpio $USR_BTN
chmod -R  775  $USR_BTN
echo "in" > $USR_BTN/direction
echo "1" > $USR_BTN/active_low

# setep lidar motor pin
echo 122 > /sys/class/gpio/export
LIDAR=/sys/class/gpio/gpio122
chgrp -HR gpio $LIDAR
chmod -R  775  $LIDAR
echo "out" > $LIDAR/direction
echo "0" > $LIDAR/active_low
echo "0" > $LIDAR/value
