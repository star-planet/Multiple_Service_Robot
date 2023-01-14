#!/bin/bash

echo ""
echo "This script copies udev rules to /etc/udev/rules.d/"
echo ""

echo "Root for USB Serial"
if [ -f "/etc/udev/rules.d/99-turtlebot3-cdc.rules" ]; then
    echo "99-turtlebot3-cdc.rules file already exist."
else
    echo 'ATTRS{idVendor}=="0483" ATTRS{idProduct}=="5740", ENV{ID_MM_DEVICE_IGNORE}="1", MODE:="0666"' > /etc/udev/rules.d/99-turtlebot3-cdc.rules
    echo 'ATTRS{idVendor}=="0483" ATTRS{idProduct}=="df11", MODE:="0666"' > /etc/udev/rules.d/99-turtlebot3-cdc.rules
    echo 'ATTRS{idVendor}=="fff1" ATTRS{idProduct}=="ff48", ENV{ID_MM_DEVICE_IGNORE}="1", MODE:="0666"' > /etc/udev/rules.d/99-turtlebot3-cdc.rules
    echo 'ATTRS{idVendor}=="10c4" ATTRS{idProduct}=="ea60", ENV{ID_MM_DEVICE_IGNORE}="1", MODE:="0666"' > /etc/udev/rules.d/99-turtlebot3-cdc.rules

    echo '99-turtlebot3-cdc.rules created'
fi

echo ""
echo "Reload rules"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger