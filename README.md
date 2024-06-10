# selfdrive_software_2025

Software for our 2025 [Intelligent Ground Vehicle Competition](http://www.igvc.org) Self Drive challenge entry. We are using [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html) on [Ubuntu 24.04](https://releases.ubuntu.com/24.04).

## Installation

If you are installing ROS2 Jazzy for the first time, ensure you are on Ubuntu 24.04 and run the followind command.
```
wget https://files.dylanzeml.in/ros/jazzy/install.sh
chmod +x install.sh
./install.sh
```

To install this repository and run it as is, clone the repository and run the following command to install the necessary dependencies.
```
chmod +x setup/all.sh
./setup/all.sh
```

## Environment

Before anything is ran or built, run the following command to install the required python packages and setup the virtual environment.
```
source environment.sh
```
