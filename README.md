# DroneFirmwareV1
ROS2 workspace to control a drone though PX4 in offboard flight mode, with and external controller.

## Getting started
We advise to use Docker to run both PX4 and the offboard application.

### 1) Install Docker
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

### 2) Clone this repository
Open a terminal, go in the folder where you want to clone this repository and run the command
```
git clone git@github.com:AVL-TMU/DroneFirmwareV1.git --recursive
```

### 3) Clone PX4 firmware
Clone the firmware from the official repository, in the same folder where you cloned this repository
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```
In order to work with the thrust and torque setpoints, you need to change a parameter in the drone sdf file.
Go into the iris model folder
```
cd PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris
```
open the file ```iris.sdf.jinja```, look for the parameter ```enable_lockstep``` and change it from 1 to 0
```
<enable_lockstep>0</enable_lockstep>
```

### 4) Clone the uXRCE-DDS client 
This is the client needed to communicate with PX4 from ROS2.
```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
```
### 5) Create the docker container
Go into the DroneFirmwareV1 folder and run the script to create the container (N.B: you must be in the DroneFirmwareV1 folder)
```
cd DroneFirmwareV1
source create_px4_container.sh 
```
Now you should be inside the container, and if you run the command ```ls```, you will see the two folders
To close the container just press ```CTRL-d```. The container will be closed but not removed, so you can start it again whenever you need. If, instead, you need to remove the container for some reason, run the command
```
docker container rm px4_ros2
```
and repeat the step (5) to create it again.

## Run the simulation
Now that everything is set up, you need to compile PX4 in order to run the simulation, start the uXRCE-DDS client and then start the controller in ROS2.
Remember that to build PX4 and to run the controller you must be inside the docker container, otherwise you will miss the dependencies to do that (to start the uXRCE-DDS client you must not be in the container if you compiled it outside the container).

### 1) Start the PX4 simulation
Considering that we will use the Gazebo GUI, in every terminal where you will start the container or open a terminal into it, first run the command
```
xhost +
```

If you closed the container, then start it with the command
```
docker start px4_ros2
docker attach px4_ros2
```
If instead you have already started the container in a terminal and want to open another terminal in the container, run the command
```
docker exec -u 0 -it px4_ros2 bash
```
In the container, go in the firmware folder and build it to start the simulation
```
cd PX4-Autopilot
make px4_sitl gazebo-classic
```
At the end of the building process you will see the Gazebo simulation opened.

### 2) Start the communication client
Open a new terminal and go into the repository and start the client
```
cd Micro-XRCE-DDS-Agent/build
./MicroXRCEAgent udp4 -p 8888
```

### 3) Start the ROS2 application
Open a new terminal in the container and, only for the first time, build the ROS2 workspace
```
xhost +
docker exec -u 0 -it px4_ros2 bash
source ../../opt/ros/foxy/setup.bash
cd DroneFirmwareV1
colcon build
```
At the end of the building process, or if you already have built before, run the commands
```
source install/setup.bash
ros2 run lee_control lee_controller
```

