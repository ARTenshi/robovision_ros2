# An Introduction to Robot Vision

We present a short introduction to Robot Vision. We first present the basic concepts of image publishers and subscribers in ROS2 and then we apply some basic commands to introduce the students to the digital image processing theory; finally, we present some RGBD and point cloud notions and applications.

For a ROS1 version of this project, please refer to [here](https://github.com/ARTenshi/robovision_ros1).

# Prerequisites

Things that you need to install the software and how to install them

```
You should have ROS2 installed.
You should have OpenCV for ROS2 installed.
```
# Installation

## 0. Requirements

Install git:

```
sudo apt-get install git
```

## 1. ROS2 Install:

### ROS2 Humble for Ubuntu 22.04:

Follow the indications here:

```
https://docs.ros.org/en/humble/Installation.html
```

### `cv_bridge`

If you already have ROS Humble installed, you can reinstall `cv_bridge` with the following command:

```
sudo apt-get install --reinstall ros-humble-cv-bridge
```

## 2. Get the introduction to robot vision libraries:

### 2.1 Clone this repository

First, create a workspace:

```
cd ~
mkdir -p robovision_ros2_ws/src
cd ~/robovision_ros2_ws/src
git clone https://github.com/ARTenshi/robovision_ros2.git
```

### 2.2 Download additional data

This project requires additional data files, available [here (Google Drive)](https://bit.ly/3WjWnI2).

Download those folders into the `~/robovision_ros2_ws/src/robovision_ros2/data/rosbags/` folder.

### 2.3 Compile your code

```
cd ~/robovision_ros2_ws
colcon build
```

#### Troubleshooting

If there are errors with `cv_bridge`, you might need to compile it from source. 

1. Install Dependencies
```
sudo apt-get update
sudo apt-get install -y python3-opencv libopencv-dev ros-humble-cv-bridge-msgs
```

2. Set Up Workspace
```
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ros-perception/vision_opencv.git
cd vision_opencv && git checkout humble
```

3. Build and Source
```
cd ~/ros2_ws
colcon build --packages-select cv_bridge
source ~/ros2_ws/install/setup.bash
```

(Optional) Add to `.bashrc`:
```
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 2.3 Test the code

Run the following command in a terminal:

```
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_images my_publisher ~/robovision_ros2_ws/src/robovision_ros2/data/images/baboon.png
```


In a second terminal, run these commands:

```
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_images my_subscriber
```

# Developing

Basic concepts on ROS2 image publishers and subscribers can be found here:

> [Lesson 1](https://github.com/ARTenshi/robovision_ros2/tree/main/robovision_1)

To learn how to process RGB images, follow the indications here:

> [Lesson 2](https://github.com/ARTenshi/robovision_ros2/tree/main/robovision_2)

To work with RGBD images, enter here:

> [Lesson 3](https://github.com/ARTenshi/robovision_ros2/tree/main/robovision_3)

Finally, how to use services and clients in ROS2 can be seen here:

> [Lesson 4](https://github.com/ARTenshi/robovision_ros2/tree/main/robovision_4)


# Authors

* **Luis Contreras** - [ARTenshi](https://artenshi.github.io/)
* **Hiroyuki Okada** - [AIBot](http://aibot.jp/)
