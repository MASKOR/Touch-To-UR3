# Introduction


# Using ROS1 on Ubunto 22 (Jammy)

Use distrobox and install Ubuntu 20

Install ROS noetic from <add link>!!!

when using distobox make sure to use
```bash
sudo hostname <Username>-Ubuntu
```
to make the distrobox hostname same as machine hostname to prevent ROS communication error

# Installation

## The Touch (3d Pen)
### Install Touch Drivers
Go into ~/catkin_ws/src/openhaptics…amd64$ or where you download the drivers from (https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US) and type sudo ./install to install further Touch device drivers. When prompted, type [y] to restart the computer

if there are any issues with the drivers check them out here (https://www.daslhub.org/unlv/wiki/lib/exe/fetch.php?media=touch_drivers_src.zip)

### Relocate Necessary Libraries
Once your computer restarts, cd into touch driver. Then cd usr/lib to go into the lib folder. Type sudo cp <libphantom.so file> /usr/lib/ to move this library into the proper path

### Initializing Device
Cd ../.. To go back 2 directories, into the TouchDriver.. directory. Plug in the Touch device into a USB port on your computer. Type sudo chmod 777 /dev/ttyACM0 to initialize device usage.

Note: initialization is required everytime the device in plugin

 To check that the Touch is being detected, type ./ListUSBHapticDevices, it should print out /dev/ttyACM0 (which means its recognized and ACM0 is the USB port that its plugged into)

 ### Adding Configuration Files
 Cd bin and run ./Touch_Setup . A menu should appear and show you the name of the device and the serial number. Just click ‘apply’ there's no need to change anything right now. This step adds configuration files necessary to operate the Touch.

 ### Source setup.bash
 be sure to sure to source the setup.bash file

### Testing / Launch RViz
When that has finished, type roslaunch omni_common omni.launch to launch RViz.

If everything is done correctly, RViz should have opened and you can see a 3D simulation of the Touch device with a blue stylus and blue head. Take the stylus out of the inkwell and you should see the simulation move with you.

If not all joints are published or if the position is fixed and only orentation is changing, change LC_NUMERIC and LANG to "en_US.UTF-8" or put LC_ALL to "en_US.UTF-8"

## Universal Robot Drivers
Look at the Documentation of UR ROS drivers to install from source

### Testing
connect with lan canle and switch to UR-PC profile for wred network
make sure EtherNet/IP is disable
```bash
run roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=192.168.1.102
```

wait for message of started controllers
start program on TP
(start and stop the program, would not work if program was paused and now resumed)
TP Program: make a new program:
	structure>URCAPS>External control
wait for
```bash
ready to recieve command
```

2nd terminal
```bash
roslaunch ur3_moveit_config moveit_planning_execution.launch
```

3rd terminal
```bash
roslaunch ur3_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur3_moveit_config)/launch/moveit.rviz
```

## Robot Controller
