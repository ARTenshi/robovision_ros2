# An Introduction to Robot Vision

We present a short introduction to Robot Vision. We first present the basic concepts of image publishers and subscribers in ROS and then we apply some basic commands to introduce the students to the digital image processing theory; finally, we present some RGBD and point cloud notions and applications.

# Prerequisites

What things you need to install the software and how to install them

```
You should have ROS installed.
You should have OpenCV installed.
You should have a working ROS workspace.
```
# Installation

## 0. Requirements

Install git:

```
sudo apt-get install git
```

## 1. ROS Install:

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
mkdir -p tidbots_ws/src
cd tidbots_ws
catkin_make
```

Then, clone this repository into the src folder:

```
cd ~/tidbots_ws/src
git clone https://gitlab.com/tidbots/robointro.git
cd ..
catkin_make
```

### 2.2 Test the code

Run the following command in a terminal:

```
roscore
```

In a second terminal, run these commands:

```
source ~/tidbots_ws/devel/setup.bash
rosrun introvision_images my_publisher ~/tidbots_ws/src/robointro/1_images/data/baboon.png
```


Then, in a third terminal, run these commands:

```
source ~/tidbots_ws/devel/setup.bash
rosrun introvision_images my_subscriber
```

# Developing

Basic concepts on ROS image publishers and subscribers can be found here:

> https://gitlab.com/tidbots/robointro/-/tree/main/1_images

To learn how process RGB images, follows the indications here:

> https://gitlab.com/tidbots/robointro/-/tree/main/2_processing

To work with RGBD images, enter here:

> https://gitlab.com/tidbots/robointro/-/tree/main/3_rgbd

# Authors

* **Luis Contreras** - [AIBot](http://aibot.jp/)
* **Hiroyuki Okada** - [AIBot](http://aibot.jp/)
