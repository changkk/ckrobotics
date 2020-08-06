# visual_tracker_2.0

This guide assumes you have OpenCV and YOLO installed (YOLO needed only for GigE detection)
Email for further queries.

Visual Tracking source code and drivers for new hardware (Gige camera and HD-Air Infinity gimbal)

## Installation:
1. Install prerequisites:
...
sudo apt-get install libspnav-dev libcwiid-dev libbluetooth-dev libusb-dev
...

2. Clone repo using following command then navigate to the root of the catkin source directory (most likely visual_tracker_2.0).
```
git clone https://github.com/haseeb7/visual_tracker_2.0
```

3. Use wstool to automatically install the remaining open source packages by running the following. This should download all supporting packages in the source directory. The tiscamera SDK will be installed in the next step. The update command can be reused to update all packages in one go if needed in the future.
```
wstool init src src/.rosinstall
wstool update -t src
```

4. Running the gigE camera requires installing the SDK from TheImagingSource. In this repo, that SDK was installed in the tiscamera directory. Use the link below to their repo and follow the procedure outlined there to install their SDK.
[TheImagingSource SDK](https://github.com/TheImagingSource/tiscamera)

5. Build the workspace.
```
catkin build
```
## Running System

Due to the transient nature of hardware and software during the development phase, there are a variety of launch files to start various system configurations. Most nodes are modular and can work independently as they all communicate via ros messages, and absence of messages will simply render nodes idle.

For now, use the following launch file to start a fully configured/equipped system. Else use the launch file to determine which nodes you want to start and comment out the rest or make a new launch file.
```
roslaunch detection full_system_starter.launch
```

## System Outline

Most of the packages in this repo and those downloaded from wstool are hardware drivers. A brief description of each is as follows.
- gimbal_controller - The gimbal driver for ROS I wrote using C++ libraries I found online. It should work with most gimbals that use SBGC serial communication. This is used to read gimbal state (IMU, encoders, etc) and send orientation or speed commands to motors.
- tiscam - ROS driver from TheImagingSource Repo with minor changes, one of which allows setting zoom based off bounding box size.
- ethz_piksi_ros - RTK GPS driver that reads position over a TCP/IP link and publishes it to a rostopic.
- usb_cam is used for the Omnicam and fisheye cameras.
- vision_opencv - ROS package for interfacing with OpenCV. Keep in mind, you need to have OpenCV installed in advance. The ROS distro version of openCV may not work in all cases.
- detection - Primary package that contains code for all visual tracking endeavors. The main nodes in that package are explained in more detail below.

## Detection Package Specifics

### detection_node.cpp
This node reads in compressed images from the omnicam and attempts to perform initial detection to obtain a cue for the gimbal. If a point of interest is detected, the angular orientation towards the region of interest is published to a topic that is then read by the governor node.
### yoloStarter
Starts up the neural network framework for real time object detection using the YOLO neural network. If an aircraft is detected, it publishes the bounding box parameters and image resolution to a ros topic. The bounding box offset from the image center is sent to a pitch and yaw PID loop that outputs a control effort as gimbal angular speed.
### governor.cpp
The governor does one of two things at a time:
   1. Standby/Cuing Mode - It reads in a cue from the detection node and cues the gimbal to reorient towards a region of interest by publishing a PoseStamped message to be read and implemented by the gimbal_controller node.
   2. Active Visual Tracking Mode - It reads in control effort (gimbal speed) from the PID nodes and publishes a ros TwistStamped message with angular velocities to be read and implemented by the gimbal_controller node.

The governor decides whether the system is in standby/cuing mode or active visual tracking mode depending on which of the two tasks it's performing. It also handles failure by running a timer since last detection before switching back to idle and waiting for a new gimbal cue from the detection node.

### broadcaster.cpp Nodes
These nodes take care of all coordinate system transforms for various moving components, as noted by the full file names of each node. For instance, aircraft_tf2_broadcaster.cpp publishes the transform from the target aircraft flight controller + RTK GPS to the base station local frame.
