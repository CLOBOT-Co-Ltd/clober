#!/bin/bash

echo ""
echo "Create udev rules for Clober"
echo "copy rule file to /etc/udev/rules.d/"
echo ""

sudo cp 99-clober.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "Reload rules"
echo ""
