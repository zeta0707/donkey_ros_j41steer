# ROS1 Donkey Servo motor + DC motor
Custom RCcar with ROS Molodic + Donkeycar!!

- Steer Servo Motor: Jetson nano J41 PWM, Caution: 3.3V

- Throttle DC Motor: Waveshare Motorhat: I2C address 0x40
---------
https://github.com/orocapangyo/donkey_ros 에서 수정하였습니다.

https://github.com/Road-Balance/donkey_ros

<p align="center">
    <img src="./Images/joy_control.gif" width="400" />
    <img src="./Images/keyboard_control.gif" width="400" />
    <img src="./Images/blob_tracking.gif" width="400" />
    <img src="./Images/yolo_control.gif" width="400" />
</p>

There's Notion Lecture Notes and Youtube video's about this project. 
But, It's written in Korean. Anyway, Here's the link

* [Notion Lecture Notes] https://www.notion.so/38a82d8f3f464a22933eef95b8a29fa4


## Tested System information

**Jetson Nano 4GB + IMX219 160**

* Ubuntu 18.04
* ROS Melodic
* Opencv4

## Packages with Brief Explanation

```
├── csi_camera => Handling Image data For IMX219 Camera  
├── donkey_control => Control RC Car with Adafruit PCA9685
├── donkey_cv => Computer Vision Package with Opencv4 
├── donkey_joy => Control RC Car with Logitech F710 Game Controller 
│
(...)
├── Images
├── LICENSE
├── README.md
```

## Prerequisite

1. Ros Packages installation
   
```bash
$ sudo apt-get install ros-melodic-cv-bridge
$ sudo apt-get install ros-melodic-image-view
```

2. OpenCV4 installation

- Can be found in [JetsonHacks repo](https://github.com/JetsonHacksNano/buildOpenCV)

3. Clone this Repo

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/Road-Balance/donkey_ros.git

$ cd ../
$ catkin_make
$ source devel/setup.bash
```

## Usage

1. csi_camera package

Packages for Image Streaming

> Check Camera Connection First!!!

```bash
gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! \
   'video/x-raw(memory:NVMM),width=3280, height=2464, framerate=21/1, format=NV12' ! \
   nvvidconv flip-method=2 ! 'video/x-raw,width=960, height=720' ! \
   nvvidconv ! nvegltransform ! nveglglessink -e
```

* `sensor_id` : this value depends on Camera Slot in Jetson Nano.

### Webcam Puslish

```bash
$ roscore

$ rosrun csi_camera webcam_pub.py
$ rosrun image_view image_view image:=/csi_image
```

### CSI Camera Publish

```bash
$ roscore

$ rosrun csi_camera csi_pub.py
$ rosrun image_view image_view image:=/csi_image
```

2. donkey_control package

Packages for controlling `RC Car` with `PCA9685` PWM driver.
You need to install `Adafruit_PCA9685` python package first 

There's four modes for controlling RC Car

* JoyStick Control
* Keyboard Control
* Blob Control
* Yolo4 Control

```bash
$ roscore

$ rosrun donkey_control joy_control.py
$ rosrun donkey_control blob_chase.py
```

3. donkey_joy package

There's two modes for using joystick

* Button mode
* Axes mode

```bash
$ roslaunch donkey_joy joy_teleop_axes.launch
$ roslaunch donkey_joy joy_teleop_btns.launch
```

4. donkey_cv package

Packages for OpenCV applications

* Find Blob with Certain color
* Publish Image location as a `geometry_msgs/Point`

```bash
$ roscore

$ rosrun donkey_cv find_ball.py
```

## Application

### **1. joy_control**

Control RC Car with game controller

<p align="center">
    <img src="./Images/joy_control.gif" width="500" />
</p>

```bash
$ roscore

# Jetson
$ rosrun donkey_control joy_control.py

# Laptop or Jetson
$ roslaunch donkey_joy joy_teleop_axes.launch
# or
$ roslaunch donkey_joy joy_teleop_btns.launch

```

### **2. keyboard_control**

Control RC Car with keyboard

<p align="center">
    <img src="./Images/keyboard_control.gif" width="500" />
</p>

```bash
$ roscore

# Jetson
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# Laptop or Jetson
$ roslaunch donkey_joy joy_teleop_axes.launch

```

### **3. blob_tracking**

Find the any color box of the Jetson Nano on the screen and change the direction of the wheel accordingly.


<p align="center">
    <img src="./Images/blob_tracking.gif" width="500" />
</p>


```bash
$ roscore

$ rosrun csi_camera csi_pub.py
$ rosrun donkey_cv find_ball.py 
$ rosrun donkey_control chase_the_ball.py 
$ rosrun donkey_control blob_chase.py 
```

Debugging with `image_view`

```bash
rosrun image_view image_view image:=/webcam_image
rosrun image_view image_view image:=/blob/image_mask
rosrun image_view image_view image:=/blob/image_blob
```

### **4. Yolo4_tracking**

Find the object of the Jetson Nano on the screen and change the direction of the wheel accordingly.


<p align="center">
    <img src="./Images/yolo_control.gif" width="500" />
</p>


```bash
#terminal #1
# camera image publish
zeta@zeta-nano:~/catkin_ws$ roslaunch jetson_csi_cam jetson_csi_cam.launch width:=416 height:=416 fps:=15

#terminal #2
#object detect using Yolo_v4
zeta@zeta-nano:~/catkin_ws$ roslaunch darknet_ros yolo_v4.launch

#terminal #3
zeta@zeta-nano:~/catkin_ws$ roslaunch donkey_control yolo_chase.launch
```
