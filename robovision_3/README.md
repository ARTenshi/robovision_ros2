# Point Cloud processing with ROS

We present an introduction to Point Cloud data in ROS and propose a simple task where the students should track a person moving in front of an RGBD camera mounted on a mobile robot.

# 0. Get the additional data

## 0.1 Rosbag data

Be sure that you have downloaded our additional data available [here (Google Drive)](https://bit.ly/3WjWnI2).

Those folders should be in the `~/robovision_ros2_ws/src/robovision_ros2/data/rosbags/` folder.

To be sure, let's test it. In one terminal run:

```bash
ros2 bag play ~/robovision_ros2_ws/src/robovision_ros2/data/rosbags/person_static --loop
```

In a second terminal, run this command:

```bash
ros2 topic list
```

You should be able to see several topics!

# 1. Getting to know your RGBD image

In this lesson, we will apply all that we have learnt in the past two units. First, let's inspect our code in the `rgbd_reader.py` file.

In the main function we can see that, as we have done before, we first initialise our node

```python
super().__init__("point_cloud_centroid")
```

and then, we subscribe to the topics we are going to use:

```python
self.rgb_subscriber_ = self.create_subscription(
    Image, "/camera/rgb/image_rect_color", self.callback_rgb_rect, 10)
self.depth_subscriber_ = self.create_subscription(
    Image, "/camera/depth_registered/image", self.callback_depth_rect, 10)
self.point_cloud_subscriber_ = self.create_subscription(
    PointCloud2, "/camera/depth_registered/points", self.callback_point_cloud, 10)
```

Here, you can notice the relevance of having our subscribers' callback functions to update our variables at different rates and a different timer function to process that information. In this case, we subscribe to three topics: an **RGB** and **Depth** images and a **Point Cloud** of 3D points, all of them come from the same RGBD sensor

<p align="center">
  <img src="images/rgbd_view.jpg" width="800">
</p>

The RGB image is a color image with three channels (Red, Green, and Blue), and the Depth image corresponds to the metric distance of each pixel of the objects in the image to an orthogonal plane that passes through the center of the camera; you can visualise it as the horizontal distance of a given point to the camera seen from above, regardless of the height, as in the Figure

<p align="center">
  <img src="images/point_depth.jpg" width="500">
</p>

We have used an `Image` topic for RGB images before. The Depth image is a matrix of floats corresponding to the metric distance in milimeters. Therefore, in the callback function `callback_depth_rect` we read it as

```python
self.depth_=bridge_depth.imgmsg_to_cv2(msg,"32FC1").copy()
self.depth_mat_ = np.array(self.depth_, dtype=np.float32)
```
As the values range from 400 (40 centimeters) to 10000 (10 meters), we normalize it to valid image values and save it in an image array to be able to display it

```python
v2.normalize(self.depth_mat_, self.depth_, 0, 1, cv2.NORM_MINMAX)
```

**Important:** the Depth information comes from a structured infrared light sensor and therefore very reflective or transparent surfaces tend to distort the depth information; those points appear as black (zero values) pixels in our depth image. The minimum distance our RGBD sensor is able to read is 40 cm, anything closer to that will be a zero value, as well.

Furthermore, from the RGB and Depth images, for every pixel in the image, we can obtain the metric XYZ position in the space -- we will not go further on this because, luckily, we see that ROS has already calculated it and the `/camera/depth_registered/points` of type `PointCloud2` provides this information. If you type

```bash
ros2 interface show sensor_msgs/msg/PointCloud2
```

in a terminal, you can see the composition of this type of message.

The point cloud message is a list of tuples (x, y, z, ...) in milimeters of the type `PointCloud2`. Therefore, in the callback function `callback_point_cloud` we read it as

```python
self.point_cloud_ = np.array(list(pc2.read_points(msg, field_names=None, skip_nans=False)))
```

This reads the point cloud as a list, so we reshape it as a matrix form aligned to our RGB image

```python
if msg.height > 1:
    self.point_cloud_ = self.point_cloud_.reshape((msg.height, msg.width, -1))
    
    rows, cols, _ = self.point_cloud_.shape
    print ('new message has arrived; point cloud size: rows: {}, cols: {}'.format(rows, cols))
```

Back to our code, we see that we have a ROS publisher where we want to publish the 3D position of an object in front of our camera; remember that a topic should have a unique name -- in this case, we called it `/object_centroid` and is of the type `Pose`:

```python
self.centroid_publisher_ = self.create_publisher(
    Pose, "/object_centroid", 10)
```

A Pose message is a geometry_msgs type that consists of a 3D position in meters and a 4D orientation in quaternion form of every point in the space with respect to the center of the camera:

- object_centroid.position.x
- object_centroid.position.y
- object_centroid.position.z
- object_centroid.orientation.x = quaternion[0]
- object_centroid.orientation.y = quaternion[1]
- object_centroid.orientation.z = quaternion[2]
- object_centroid.orientation.w = quaternion[3]

In the PointCloud2 message, the axes are as follows:

- x: positive from the center of the camera to the right
- y: positive from the center of the camera to the bottom
- z: positive from the center of the camera to the front

With the origin at the center of the camera, the XY axes (front view) and the `ROLL` angle of a point `p` on the plane are:

<p align="center">
  <img src="images/xy_view.jpg" width="500">
</p>

the YZ axes (side view) and the `PITCH` angle of a point `p` on the plane are

<p align="center">
  <img src="images/yz_view.jpg" width="500">
</p>

and the XZ axes (top view) and the `YAW` angle of a point `p` on the plane are:

<p align="center">
  <img src="images/xz_view.jpg" width="500">
</p>

### Homework 1.1

Please, inspect the `rgbd_reader.cpp` implementation. The structure is very similar in C++ and Python, so you can use any of them, depending on your requirements.


# 2. Point Cloud's manipulation

## 2.1 Single element access

Now, we will process this information in a timer function `point_cloud_processing`, in Python

```python
self.processing_timer_ = self.create_timer(0.030, self.point_cloud_processing)
```

and in C++

```cpp
processing_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(30),
    std::bind(&PointCloudCentroidNode::point_cloud_processing, this));
```

We access a single element in our array just as any array in Python `self.point_cloud_[row_id, col_id, 0]`. To access a single dimension X, Y, or Z, we can indicate it directly `self.point_cloud_[row_id, col_id, 0][XYZ]`, where XYZ=0 for the dimension X, XYZ=1 for the dimension Y, and XYZ=2 for the dimension Z. In our example, to access the 3D information in the central point of our image we enter

```python
rows, cols, _= self.point_cloud_.shape
row_id = int(rows/2)
col_id = int(cols/2)

p = [float(self.point_cloud_[row_id, col_id, 0][0]), 
     float(self.point_cloud_[row_id, col_id, 0][1]), 
     float(self.point_cloud_[row_id, col_id, 0][2])]
```

In C++, we access an element in out matrix as `point_cloud_.at<cv::Vec4f>(row_id, col_id)`, and each component as `point_cloud_.at<cv::Vec4f>(row_id, col_id)[XYZ]`, where XYZ=0 for the dimension X, XYZ=1 for the dimension Y, and XYZ=2 for the dimension Z

```cpp
int rows = point_cloud_.rows;
int cols = point_cloud_.cols;
int row_id = rows / 2;
int col_id = cols / 2;

cv::Vec4f point = point_cloud_.at<cv::Vec4f>(row_id, col_id);
```

Finally, we store it in our global variable to be published later in the program's main loop by our ROS publisher. In Python

```python
self.centroid_=Pose()

self.centroid_.position.x = p[0]
self.centroid_.position.y = p[1]
self.centroid_.position.z = p[2]

self.centroid_.orientation.x=0.0
self.centroid_.orientation.y=0.0
self.centroid_.orientation.z=0.0
self.centroid_.orientation.w=1.0

#Publish centroid pose
self.centroid_publisher_.publish(self.centroid_)
```

and in C++

```cpp
geometry_msgs::msg::Pose centroid;

centroid.position.x = static_cast<float>(point[0]); // x
centroid.position.y = static_cast<float>(point[1]); // y
centroid.position.z = static_cast<float>(point[2]); // z

centroid.orientation.x = 0.0;
centroid.orientation.y = 0.0;
centroid.orientation.z = 0.0;
centroid.orientation.w = 1.0;

// Publish the centroid pose
centroid_publisher_->publish(centroid);
```

## 2.2 Run your code

Now, let's try our code. So, run the following commands:

```bash
cd ~/robovision_ros2_ws
colcon build
```

If you don't have an RGBD camera, don't worry, we provide you with a ROS bag with some data collected using an XTion Pro mounted at the top of a Turtlebot 2, at a height 1.0 meter from the floor. In a different terminal, run:

```bash
ros2 bag play ~/robovision_ros2_ws/src/robovision_ros2/data/rosbags/person_static --loop
```

Then, in a different terminal enter

```bash
ros2 topic list
```

Can you see all the different topics you can work with!?

Now enter, for a Python implementation

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_rgbd rgbd_reader.py
```

or, for C++ implementation

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_rgbd rgbd_reader
```

Can you see the 3D information of our middle point in the image?

Finally, in a new terminal:

```bash
ros2 topic info /object_centroid
```

and, then 

```bash
ros2 topic echo /object_centroid
```

You should be able to see the same information being published in our ROS topic!

### Homework 2.1

* Provide the 3D position of five different points in our image (enter different row_id, and col_id). Can you see how the X, Y, and Z values change with respect to the central point? X is positive to the right of our middle point and Y is positive below the middle point.

* What's the 3D information for point (row_id=0, col_id=0)? Please note that, when the information is not available for a given point due to the structured light reflection properties, the system returns 'nan' values. In Python you can check if a variable is `nan` with the `math.isnan()` function in the `math` library -- this function returns a `True` or `False` value. You can validate your data using the `if ( not math.isnan(( self.point_cloud_[row_id, col_id, 0][0] ) ):` structure, for example. Similarly, in C++ we have `std::isnan()`.

# 3. Final project

Now you have all the tools to program a nice robot vision project.

The problem is this: We have a robot facing towards a moving person and **we want to find the relative position of this person to the center of the robot (represented here as the camera position).** 

In other words, we want to find the 3D position of person_1

<p align="center">
  <img src="images/scene_full.jpg" width="500">
</p>

How can you do it?

You can use the `person_static` and the `person_dynamic` ROS bags in the `~/robovision_ros2_ws/src/robovision_ros2/data/rosbags/` folder if you don't have an RGBD camera.

**Hint** Remember that the Point Cloud returns all the 3D information of all points visible by the camera, so why not limit it? You can create a valid zone where you can track your person better without any extra information

<p align="center">
  <img src="images/scene_box.jpg" width="500">
</p>

You can use a structure where you compare if every image 3D point is inside this box (first check if your point is not `nan`)

<p align="center">
  <img src="images/scene_crop.jpg" width="500">
</p>

Don't forget to use the appropriate signs, especially in the Y-axis, if the camera is 1.0 meter above the floor, what condition should any point different than the floor meet? Again, pay special attention to the Y-axis direction.

**Hint** Remember the `ROLL`, `PITCH` and `YAW` definitions

<p align="center">
  <img src="images/scene_person.jpg" width="500">
</p>

You can convert those values to quaternion form by using the Python function

```python
from tf_transformations import quaternion_from_euler
quaternion = quaternion_from_euler(roll, pitch, yaw)
```

and then store it in our variable

```python
object_centroid.orientation.x = quaternion[0]
object_centroid.orientation.y = quaternion[1]
object_centroid.orientation.z = quaternion[2]
object_centroid.orientation.w = quaternion[3]
```

Similarly, in C++

```cpp
tf2::Quaternion quaternion;
quaternion.setRPY(roll, pitch, yaw);
```

and

```cpp
centroid.orientation.x = quaternion.x();
centroid.orientation.y = quaternion.y();
centroid.orientation.z = quaternion.z();
centroid.orientation.w = quaternion.w();
```

Now, try implementing it in Python and C++. Good luck!

### Challenge 3.1

How can you improve the performance? **Important: Remember that each 2D image point has its corresponding 3D point in the Point Cloud, you can use this information!** Maybe you can try detecting the person's face first and then take the 3D points average inside that region. OpenCV provides a series of functions that might help you with that, e.g.

> https://docs.opencv.org/3.4/db/d28/tutorial_cascade_classifier.html 

What about detecting the whole person's body and getting the 3D points average inside the person's bounding box? In the example above, you can add an extra classifier of the form:

> bodydetection = cv2.CascadeClassifier('cascades/haarcascade_fullbody.xml')

Would you accept the challenge?

## Authors

* **Luis Contreras** - [ARTenshi](https://artenshi.github.io/)
* **Hiroyuki Okada** - [AIBot](http://aibot.jp/)
