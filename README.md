# Bebop 2 multi UAV

This repository contains software for position control of **Parrot Bebop 2 drone**. The system enables to control the real drone or simulated in **Parrot Sphinx** with the use of **bebop_autonomy** driver. Also, the following software enables to run 2 or 3 drones and control them as a formation, where one of them is a leader and the rest are followers. The solution is based on **Robot Operating System**.

## Results in simulation and real tests
The position of each drone is controlled by a PID controller, which was tuned manually first and then fine-tuned with the iterative autotuning method based on golden-section search. Images below show the formation of 3 drones in the Gazebo simulator. The UAV formation follows the trajectory around the observed object (in this example - the chair). Each drone is focused on the object during the whole flight.

<p float="left">
<img src=".\images\gazebo1.png" alt="Sim1" height="240" />
<img src=".\images\gazebo2.png" alt="Sim2" height="240" />
</p>

There is also a simple collision avoidance system that prevents drones from colliding. It is based on setting safety zones around each UAV.

We also tested the system in real conditions. For this purpose we used a motion capture system (Optitrack) to obtain actual position of each drone. Example of one drone and two drones flights:

<p float="left">
<img src=".\images\lab1.png" alt="Lab1" height="240" />
<img src=".\images\lab2.png" alt="Lab2" height="240" />
</p>

Video on YouTube: https://youtu.be/OJhgMnpAi4s

___
## System requirements
- Ubuntu 16.04
- ROS Kinetic
> It was not tested with newer software yet.


## Installation
The first step is to install **Parrot Sphinx** - [installation procedure.](https://developer.parrot.com/docs/sphinx/installation.html)

Then install **bebop_autonomy** - [installation procedure.](https://bebop-autonomy.readthedocs.io/en/latest/installation.html)

Download the repository:
```
git clone https://github.com/BartlomiejKulecki/bebop2_multi_uav.git
```

Copy the src/formation_controller and src/position_controller folders to ~/bebop_ws/src directory:
```
cd bebop2_multi_uav/src
cp * ~/bebop_ws/src/
```
Install dependencies:
```
sudo apt-get install libncurses5-dev libncursesw5-dev
```
And compile:
```
cd ~/bebop_ws
catkin_make
```

Simulation of multiple UAVs does not work with the newest bebop2 firmware, so [download](http://plf.parrot.com/sphinx/firmwares/ardrone3/milos_pc/4.4.2/images/ardrone3-milos_pc.ext2.zip) version 4.4.2 and save it in a directory:  */home/username/firmware*

Copy files:
```
cd bebop2_multi_uav/files
cp drones/*.drone  /opt/parrot-sphinx/usr/share/sphinx/drones
cp models/*  /opt/parrot-sphinx/usr/share/sphinx/models
cp worlds/*.world  /opt/parrot-sphinx/usr/share/sphinx/worlds
cp launch/*.launch  ~/bebop_ws/bebop_autonomy/bebop_driver/launch
```
In copied drone files correct firmware path (your username).


## Running simulation
Before starting the simulation, you should go to the folder 
`~/bebop_ws/src/position_controller/src` and `~/bebop_ws/src/formation_controller/src`, copy the appropriate file / files from the directories there (depending on the number of drones, more information in the readme files located there).\
You can also change some params in file: `~/bebop_ws/src/formation_controller/config/params.yaml `

> To run the N drone simulation, you must have N network interfaces, e.g. for three drones you can use a built-in WiFi card, an additional USB WiFi dongle, and an Ethernet port.

Next, compile the code:
```
cd ~/bebop_ws
catkin_make
```
Next, run:
```
sudo firmwared
```
Check your network interfaces:
```
iwconfig
```
This command lists the available network interfaces and their names (e.g. wlan0). The name of the network you use should be entered in the appropriate .drone file in the stolen interface field as follows:
```xml
<stolen_interface> your_network_name:eth0:192.168.42.1/24 </stolen_interface>
```
eth0 and the IP address should be left unchanged.

Next, run sphinx:
```
# 1 drone:
sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2_local.drone  
# 2 drones:
sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2_local.drone /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2_local2.drone
# 3 drones:
sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2_local.drone  /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2_local2.drone /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2_local3.drone
```
If you want to select another world for simulation:
```
sphinx /opt/parrot-sphinx/usr/share/sphinx/worlds/room_chair.world /opt/parrot-
sphinx/usr/share/sphinx/drones/bebop2_local.drone
```

In bebop_node.launch files, the bebop_ip field should be assigned the ip_sphinx variable.

Next (in new terminal) run driver for each drone:
```
source ~/bebop_ws/devel/setup.bash
roslaunch bebop_driver  bebop_node.launch
roslaunch bebop_driver  bebop_node_f1.launch
roslaunch bebop_driver  bebop_node_f2.launch
```
position controller (in a new terminal):
```
source ~/bebop_ws/devel/setup.bash
rosrun  position_controller  position_controller
```
and formation controller (in a new terminal):
```
source ~/bebop_ws/devel/setup.bash
roslaunch  formation_controller  formation_ctrl.launch
```
You can use a keyboard to send commands: takeoff, land, emergency stop, start trajectory (instructions will show in terminal).

## Running real drones
>**WARNING!**\
>Be especially careful! The author does not guarantee safety while using this software on real drones!

Before starting, you should go to the folder `~/bebop_ws/src/position_controller/src` and `~/bebop_ws/src/formation_controller/src`, copy the appropriate file / files from the directories there (depending on the number of drones, more information in the readme files located there). \
You can also change some params in file: `~/bebop_ws/src/formation_controller/config/params.yaml`

>If you want to run a few drones at the same time you have to connect to each by WiFi (multiple WiFi interfaces are needed). Each drone needs different IP (you should set it).

In bebop_node.launch files, the bebop_ip field should be assigned the ip_real variable (it should match to IP set on the drone).

Connect to drones WiFi and run driver for each drone (in a new terminal):
```
source ~/bebop_ws/devel/setup.bash
roslaunch bebop_driver  bebop_node.launch
roslaunch bebop_driver  bebop_node_f1.launch
roslaunch bebop_driver  bebop_node_f2.launch
```
position controller (in a new terminal):
```
source ~/bebop_ws/devel/setup.bash
rosrun  position_controller  position_controller
```
and formation controller (in a new terminal):
```
source ~/bebop_ws/devel/setup.bash
roslaunch  formation_controller  formation_ctrl.launch
```
You can use a keyboard to send commands: takeoff, land, emergency stop, start trajectory (instructions will show in terminal).
