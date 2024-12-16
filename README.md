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
Go into ~/catkin_ws/src/openhaptics…amd64$ and type sudo ./install to install further Touch device drivers. When prompted, type [y] to restart the computer

### Relocate Necessary Libraries
Once your computer restarts, cd into touch driver. Then cd usr/lib to go into the lib folder. Type sudo cp <libphantom.so file> /usr/lib/ to move this library into the proper path

### Initializing Device
Cd ../.. To go back 2 directories, into the TouchDriver.. directory. Plug in the Touch device into a USB port on your computer. Type sudo chmod 777 /dev/ttyACM0 to initialize device usage.

Note: initialization is required everytime the device in plugin

 To check that the Touch is being detected, type ./ListUSBHapticDevices, it should print out /dev/ttyACM0 (which means its recognized and ACM0 is the USB port that its plugged into)

 ### Adding Configuration Files
 Cd bin and run ./Touch_Setup . A menu should appear and show you the name of the device and the serial number. Just click ‘apply’ there's no need to change anything right now. This step adds configuration files necessary to operate the Touch.

 ### Editing .cpp Files
 Next, we need to locate two .cpp files and make a minor change. Both are located through the path (/catkin_ws/src/phantom_omni/omni_common/src). gedit the omni.cpp file first. Use ctrl+f and type in “hHD =” this should find only one reference. change this line to read “hHD = hdInitDevice(“Default Device”) *This is case and space sensitive* save the file using ctrl+s and close the window.

 Go into the omni_state.cpp file and repeat above step. Make sure to save the changes

 ### Edit omni.lauch File
 Go back to your terminal and change path to (/catkin_ws/src/phantom_omni/omni_common/launch). Gedit omni.launch to open the file. On line 3, the last few words should say type=”state_publisher” to type=”robot_state_publisher”. (Just add ‘robot_’ within the parentheses). Make sure to save the changes.

 ### Source setup.bash
 be sure to sure to source the setup.bash file

### Testing / Launch RViz
When that has finished, type roslaunch omni_common omni.launch to launch RViz.

If everything is done correctly, RViz should have opened and you can see a 3D simulation of the Touch device with a blue stylus and blue head. Take the stylus out of the inkwell and you should see the simulation move with you.

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
