# DroneFirmwareV1
ROS2 workspace to control a drone though PX4 in offboard flight mode, with and external controller.

## Getting started
We advise to use Docker to run both PX4 and the offboard application.

### 1) Clone this repository
Open a terminal, go in the folder where you want to clone this repository and run the command
```
git clone git@github.com:AVL-TMU/DroneFirmwareV1.git --recursive
```

### 2) Install Docker
There is a convinient script that facilitate everything
```
curl -fsSL get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```
We suggest to use docker as a non-root user, that way your build folder won't be owned by root after using docker. In a new terminal run
```
# Create docker group (may not be required)
sudo groupadd docker
# Add your user to the docker group.
sudo usermod -aG docker $USER
```
**Now log out and in again before using docker!!**

### 3) Clone PX4 firmware
Clone the firmware from the official repository, in the same folder where you cloned this repository
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```


### TO DO:
1. finish the guide to get start
2. add the info to use Microxrcedds
3. write the commands to run the simulation with docker
