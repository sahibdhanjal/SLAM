#!/bin/bash

groupadd gpio
usermod -a -G gpio variscite
usermod -a -G gpio mbot
chmod +x gpio_init.sh
cp ./gpio_init.sh /etc/init.d
update-rc.d gpio_init.sh defaults 01 99
./gpio_init.sh

echo "Please log out and log back in for changes to be applied properly."