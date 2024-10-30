#!/bin/bash

# Check if vectorsecrets.txt exists
if [ ! -f vectorsecrets.txt ]; then
    echo "vectorsecrets.txt does not exist. Creating it now:"
    echo -n "Please enter your login: "
    read username
    echo -n "Please enter your password: "
    read -s password
    echo -n "machine files.dylanzeml.in login $username password $password" > vectorsecrets.txt
fi

sudo apt update
sudo apt install wget unzip -y

# Vectornav Dependencies
bash ./vnav.sh

# Python deps
sudo apt install python3-pip -y
python3 -m pip config set global.break-system-packages true
sudo python3 -m pip config set global.break-system-packages true
pip3 install python-can[serial]
pip3 install websockets

# Copy the udev rules to the correct location
sudo cp etc/selfdrive.rules /etc/udev/rules.d/selfdrive.rules

# Reload udev
sudo service udev reload
sleep 2
sudo service udev restart

# Copy services
sudo cp etc/selfdrive_service /etc/systemd/system/selfdrive.service
sudo cp etc/selfdrive_service.sh /usr/bin/selfdrive_service.sh

# chmod time :D
sudo chmod +x /usr/bin/selfdrive_service.sh
sudo chmod 644 /etc/systemd/system/selfdrive.service

# boot time
echo "Use 'systemctl enable selfdrive' to enable the service at boot time."