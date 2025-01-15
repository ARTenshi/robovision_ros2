# An Introduction to Robot Vision

We present a short introduction to Robot Vision. We first present the basic concepts of image publishers and subscribers in ROS1 and then we apply some basic commands to introduce the students to the digital image processing theory; finally, we present some RGBD and point cloud notions and applications.

For a ROS2 version of this project, please refer to [here](https://github.com/ARTenshi/robovision_ros2).

# Prerequisites

Things that you need to install the software and how to install them

```
You should have ROS1 installed.
You should have OpenCV for ROS1 installed.
```
# Installation

## 0. Requirements

Install git:

```
sudo apt-get install git
```

## 1. ROS1 Install:

### Ubuntu 20.04:

Follow the indications here:

```
https://wiki.ros.org/noetic/Installation/Ubuntu
```


## 2. Get the introduction to robot vision libraries:

### 2.1 Clone this repository

First, create a workspace:

```
cd ~
mkdir -p robovision_ros1_ws/src
cd robovision_ros1_ws
catkin_make
```

Then, clone this repository into the src folder:

```
cd ~/robovision_ros1_ws/src
git clone https://github.com/ARTenshi/robovision_ros1.git
cd ..
catkin_make
```

### 2.2 Download additional data

This project requires additional data files, available [here (Google Drive)](https://bit.ly/3PzJp5m).

Download those files into the `~/robovision_ros1_ws/src/robovision_ros1/data/rosbags/` folder.


### 2.3 Test the code

Run the following command in a terminal:

```
roscore
```

In a second terminal, run these commands:

```
source ~/robovision_ros1_ws/devel/setup.bash
rosrun introvision_images my_publisher ~/robovision_ros1_ws/src/robovision_ros1_ws/data/images/baboon.png
```


Then, in a third terminal, run these commands:

```
source ~/robovision_ros1_ws/devel/setup.bash
rosrun introvision_images my_subscriber
```

# Developing

Basic concepts on ROS1 image publishers and subscribers can be found here:

> [Lesson 1](https://github.com/ARTenshi/robovision_ros1/tree/main/1_images)

To learn how to process RGB images, follow the indications here:

> [Lesson 2](https://github.com/ARTenshi/robovision_ros1/tree/main/2_processing)

To work with RGBD images, enter here:

> [Lesson 3](https://github.com/ARTenshi/robovision_ros1/tree/main/3_rgbd)

# Authors

* **Luis Contreras** - [ARTenshi](https://artenshi.github.io/)
* **Hiroyuki Okada** - [AIBot](http://aibot.jp/)
