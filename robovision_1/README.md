# Image Publishers and Subscribers in ROS2

The goal of this repository is to introduce students to image publishers and subscribers using ROS2 and OpenCV.

# 0 CMakeList.txt and package.xml

When we build and install a package in ROS2, we need to give the compiler some information regarding names and libraries. This guide explains how to use the `CMakeLists.txt` and `package.xml` files to configure and build a package in ROS2.

## 0.1 `package.xml`
The `package.xml` file contains package metadata and dependency declarations.

- **Key Sections**:
  - `<name>`: Defines the package name (`robovision_images`).
  - `<version>`: Specifies the package version.
  - `<maintainer>`: Provides the maintainer and contact email.
  - `<license>`: Placeholder for the license.
  - `<buildtool_depend>`: Tools for building the package, such as `ament_cmake` and `ament_cmake_python`.
  - `<depend>`: Runtime dependencies like `rclcpp`, `cv_bridge`, `sensor_msgs`, and `std_msgs`.
  - `<test_depend>`: Dependencies for testing, such as `ament_lint_auto`.

## 0.2 `CMakeLists.txt`
The `CMakeLists.txt` file defines how the package is built.

- **Key Components**:
  - Specifies the minimum CMake version and compiler options.
  - Locates dependencies (`rclcpp`, `cv_bridge`, `OpenCV`, etc.).
  - Declares include directories for OpenCV and `cv_bridge`.
  - Adds executables for C++ nodes like `my_publisher` and `my_camera_publisher` and links their dependencies.
  - Installs Python scripts and C++ executables to the appropriate directories.
  - Integrates the package with ROS2 using `ament_package()`.

## 0.3 Example: `robovision_images` 

### 0.3.1 `package.xml`

We declare the name of our project

```xml
<name>robovision_images</name>
```

We are using both C++ and Python code in our project, and therefore, we need to declare it:

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>ament_cmake_python</buildtool_depend>

<depend>rclcpp</depend>
<depend>rclpy</depend>
```

Additionally, we need to add all the ros-based dependencies we will use in our project

```xml
<depend>cv_bridge</depend>
<depend>sensor_msgs</depend>
<depend>std_msgs</depend>
```

### 0.3.2 `CMakeList.txt`

First, we declare the name of our project

```cmake
project(robovision_images)
```

Similarly, we need to declare that we are using both C++ and Python code

```cmake
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
```
and the dependencies we are using

```cmake
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
```

To build a project, we need to specify which files we want to compile and how we are going to refer them to our package. For C++ files, we declare them as in the example here

```cmake
add_executable(my_publisher src/my_publisher.cpp)
ament_target_dependencies(my_publisher 
    rclcpp 
    cv_bridge 
    sensor_msgs 
    OpenCV
)
```

and for Python, we use

```cmake
ament_python_install_package(${PROJECT_NAME})
```

Finally, we need to install them in our install folder to be able to source them later

```cmake
install(TARGETS
  my_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/my_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)
```

This is a brief introduction to how to use these files to configure your project. Please inspect those files in each section!

# 1. ROS Publishers and Subcribers

We assume the students have a notion of these subjects. If it is not the case, they can start here, for C++:

> https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

and here, for Python:

> https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

ROS2 is designed for object-oriented programming (OOP), and therefore we will provide solutions based on such paradigm (i.e. Class-based templates).

The basic **C++** structure is as follows:

```cpp
#include "rclcpp/rclcpp.hpp"

class MyCustomNode : public rclcpp::Node // CHANGE CLASS NAME
{
public:
    MyCustomNode() : Node("node_name") // CHANGE CLASS AND NODE NAME
    {
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>(); // CHANGE CLASS NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

While the basic **Python** structure is:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyCustomNode(Node): # CHANGE CLASS NAME
    def __init__(self):
        super().__init__("node_name") # CHANGE NODE NAME


def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode() # CHANGE CLASS NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```


# 1.1 Static image publisher

Have a look at the `my_publisher.cpp` file. This file presents the basic structure to construct a **publisher** in ROS2.

## 1.1.1 Create a publisher

The `main` section remains the same, and we use it to call our class; however, we have modified it to get some arguments. Within our `ImagePublisherNode` class, first, we need to create a node object and give it a unique name:

```cpp
ImagePublisherNode(const std::string & image_path) : Node("image_publisher")
```

Then, we create a ROS2 publisher and give it a unique name to be shown as a ROS2 topic, so anyone else can access it without confusion. The structure is simple: create_publisher<msg::Type>("topic/name", 10). In this case, the type is `sensor_msgs::msg::Image` and the name of our output topic is `camera/image`, as follows:

```cpp
image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);

```

Now, let's gather some data. We will open an image using OpenCV and then publish it in our topic. First, we read the image:

```cpp
cv::Mat cv_image = cv::imread(image_path, cv::IMREAD_COLOR);
if (cv_image.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load image from path: %s", image_path.c_str());
    return;
}
```

and we convert it to a **ROS message**, the type of data that can be sent through the ROS framework:

```cpp
image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg();
```

Now we can publish this message. 

We can do it once (`image_publisher_->publish(*image_msg_)`), but it means that the information will be only available for a fraction of time; if you don't access it at that very moment, you won't be able to use it anymore. So, let's publish it all the time! We first decide on a frame rate, which is the number of **frames per second** (fps). In general, we consider **real-time** something around 30 fps. We use a `while` loop to publish our image at 30 Hz (i.e. 30 fps). In ROS2, we need to create a **tiner**, that will iterate at a given frequency once the node starts to spin in the `main` section:

```cpp
image_timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
				       std::bind(&ImagePublisherNode::publish_image, this));
```

This **timer** will call a function (`ImagePublisherNode::publish_image`) where we can publish our image:

```cpp
void publish_image()
{
    image_publisher_->publish(*image_msg_);
}
```

Finally, notice that we need to define our variables; in particular, in C++, our ROS2 variables are all shared pointers and we need to define them as such:

```cpp
sensor_msgs::msg::Image::SharedPtr image_msg_;

rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
rclcpp::TimerBase::SharedPtr image_timer_;
```

And that's it, we have our first ROS2 publisher. 

## 1.1.2 Test your code

Run the following command in a terminal:

```bash
ros2 topic list
```

You should see something like:

```bash
/parameter_events
/rosout
```

Now, in the same terminal, run the following commands:

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_images my_publisher ~/robovision_ros2_ws/src/robovision_ros2/data/images/baboon.png
```

Then, in a new terminal, run this command again:

```bash
ros2 topic list
```

Do you remember this line `image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);`? Well, now we can see our topic `/camera/image`! Furthermore, we can get the details of it. If we enter the command:

```bash
ros2 topic info /camera/image
```

we can see:

```bash
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 0

```

Finally, enter:

```bash
ros2 node list
```

and

```bash
ros2 node info /image_publisher
```

we see:

```bash
image_publisher
  Publishers:
    /camera/image: sensor_msgs/msg/Image
```

So, we can see that our topic is a *sensor_msgs/msg/Image* data and that the *Publisher* corresponds to the name we gave it when we defined the node class a few lines above `ImagePublisherNode(const std::string & image_path) : Node("image_publisher")`. However, we don't have any *Subscriber* yet. Let's solve it in Section 1.3.

## 1.1.3 Homework 1.1

* Add a new publisher in your code that publishes a scaled version by half of the original image.

To give you an idea of how ROS2 works, we will help you to solve this task this time, but you are expected to solve it all by yourself.

Do you remember our ROS2 publisher? We need one publisher per topic, so let's add a new one (don't forget to give a different and unique name to each topic, in this case, we named it `camera/scaled_image`):

```cpp
scaled_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/scaled_image", 10);
```

Don't forget to define it:

```cpp
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr scaled_image_publisher_;
```

After a short search on the internet, we found that, to scale an image in OpenCV, we can use the following command:

```cpp
cv::Mat scaled_image;
cv::resize(cv_image, scaled_image, cv::Size(), 0.5, 0.5, CV_INTER_AREA);
```

Now, we need to create a new image message:

```cpp
scaled_image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", scaled_image).toImageMsg();
```

and publish our scaled image:

```cpp
scaled_image_publisher_->publish(*scaled_image_msg_);
```

Your code should look something like:

```cpp
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class ImagePublisherNode : public rclcpp::Node
{
public:
    ImagePublisherNode(const std::string & image_path) : Node("image_publisher")
    {
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
        scaled_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/scaled_image", 10);

        // Load the image from file
        cv::Mat cv_image = cv::imread(image_path, cv::IMREAD_COLOR);
        if (cv_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image from path: %s", image_path.c_str());
            return;
        }

        //Resize your image
        cv::Mat scaled_image;
        cv::resize(cv_image, scaled_image, cv::Size(), 0.5, 0.5, CV_INTER_AREA);

        //Convert the output data into a ROS message format
        image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg();
        scaled_image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", scaled_image).toImageMsg();

        image_timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
                                               std::bind(&ImagePublisherNode::publish_image, this));
        
        RCLCPP_INFO(this->get_logger(), "Starting image_publisher application in cpp...");
    }

private:
    
    sensor_msgs::msg::Image::SharedPtr image_msg_, scaled_image_msg_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr scaled_image_publisher_;
    rclcpp::TimerBase::SharedPtr image_timer_;

    void publish_image()
    {
        image_publisher_->publish(*image_msg_);
        scaled_image_publisher_->publish(*scaled_image_msg_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: my_publisher <image_path>");
        rclcpp::shutdown();
        return 1;
    }

    std::string image_path = argv[1];

    auto node = std::make_shared<ImagePublisherNode>(image_path);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
```

Now let's test it!

First, we need to compile our code, in a new terminal run:

```bash
cd ~/robovision_ros2_ws
colcon build
```

Now, run the following command:

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_images my_publisher ~/robovision_ros2_ws/src/robovision_ros2/data/images/baboon.png
```

Finally, in a new terminal, run this command:

```bash
ros2 topic list
```

and

```bash
ros2 node info /image_publisher
```

What can you see? Please, explain.


# 1.2 Camera publisher

Have a look at the `my_camera_publisher.cpp` file. This file presents the basic structure to open a camera and create a **publisher** in ROS2. Can you note the similarities and differences between static image and camera publishers?

## 1.2.1 Create a publisher

First, we started our node as before:

```cpp
image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
```

Please note that we use the same topic name for the static image and the camera publishers, so you can only use one of them at a time.

Let's open our camera! Every camera connected to your computer has an ID number, starting from 0. If you have a laptop with an extra USB camera attached to it, the laptop's camera will have ID 0 and the USC camera ID 1, and so on. In any case, provided you have at least one camera connected to your computer, there will be a device with ID 0 and, therefore, the default value is zero. We may have a hardcoded ID value, or receive it as a dynamic argument when running our node; the latter case is preferred so, let's do it.

Now, remember how we enter the image path in our previous code, in the `main` function:

```cpp
if (argc != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: my_publisher <image_path>");
    rclcpp::shutdown();
    return 1;
}

std::string image_path = argv[1];
```

However, ROS has a built-in option to enter dynamic arguments, and we will use it from now on -- this will be useful when creating launch files. To do this, in our class definition, for a camera_index variable, we declare:

```cpp
// Declare and get the camera index parameter (default is 0)
this->declare_parameter<int>("camera_index", 0);
int camera_index = this->get_parameter("camera_index").as_int();
```

Then, the following lines should open a camera with ID camera_index:

```cpp
// Open the camera
capture_.open(camera_index);

// Check if the camera opened successfully
if (!capture_.isOpened())
{
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
    throw std::runtime_error("Camera not available");
}
```

Now, we should notice that, unlike static images, video sequences change in time and therefore we should update our frame at a desired frame rate (depending on your device specifications). Therefore, we need to get a new frame in our timer callback function (`capture_ >> cv_image;`) and publish it. In general, a camera takes some time to start after you call it, so we need to wait until our program starts receiving a video stream (we use the `!capture_.isOpened()` to do that). Then, our code to read and publish camera images is:

```cpp
void publish_image()
{
    cv::Mat cv_image;
    capture_ >> cv_image; // Capture an image frame

    if (cv_image.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Empty frame captured");
        retu0rn;
    }

    //Convert the output data into a ROS message format
    sensor_msgs::msg::Image::SharedPtr image_msg_;
    image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg();
    image_publisher_->publish(*image_msg_);
}
```

## 1.2.2 Test your code

First, we need to compile our code, in a new terminal run:

```bash
cd ~/robovision_ros2_ws
colcon build
```

Now, run the next command:

```bash
ros2 topic list
```

You should see something like this:

```bash
/parameter_events
/rosout
```

Now, in the same terminal, run the following commands:

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_images my_camera_publisher
```

Then, again, in a new terminal, run this command:

```bash
ros2 topic list
```

Can you explain the output? How do you get extra information from your topic?

If you want to test more than one camera, you can specify using the `--ros-args` and, for a parameter declaration, `-p param_name:=param_value`; in this case, we set the `camera_index` variable with your camera index (as an example, we declare it as 0):

```bash
ros2 run robovision_images my_camera_publisher --ros-args -p camera_index:=0
```

## 1.2.3 Homework 1.2

* Add a new publisher in your code that publishes a scaled version by half of the original video frame.

**Hint** You should start your new publisher outside the timer function `publish_image()` but process the scaled image inside it.


# 1.3 Image subscriber

Now that we have a stream of images being published in ROS, let's do something with them. Have a look at the `my_subscriber.cpp` and `my_subscriber.py` files. The important part here is the **callback function**. Again, review your concepts on ROS publishers and subscribers using the links provided before.

## 1.3.1 Create a subscriber

### C++

As with any node in ROS, we need to start it and give it a unique name:

```cpp
ImageSubscriberNode() : Node("image_subscriber")
```

Now, again, we need to create a ROS subscriber. The basic structure is: `create_subscription<msg::Type>("topic/name", queue_size, callback_function)`; the `msg::Type` is of the same type as the topic we want to get. Also, we tell ROS which function will be called every time a new image arrives. In our example, our `camera/image` topic is of type `sensor_msgs::msg::Image` and we call a `callback_image`, we declare it with a std::bind (don't worry about it, just use this structure every time you subscribe to a topic!). Then we have:

```cpp
image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/image", 10, std::bind(&ImageSubscriberNode::callback_image, this, std::placeholders::_1));
```

That was easy... but now we need to create the callback function. The basic structure consists of a function's name and, as a parameter, the variable with the data type that will be entered. In this case, our `msg` variable is of type `sensor_msgs::msg::Image::SharedPtr`:

```cpp
void callback_image(const sensor_msgs::msg::Image::SharedPtr msg)
```

Inside this function, we can process our data. However, we prefer to use our subscribers callback functions to update our variables and process them in a custom function (e.g. our timer function); this is because, in compĺex programs, we might be required to process the information from several topics at a time.

Therefore, first, let's focus on our callback function. We first need to transform our `sensor_msgs::msg::Image::SharedPtr` data in the `msg` variable to `cv::Mat`:

```cpp
void callback_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
    image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    is_image_ = true;
}
```

And then, we can use it in our timer function `image_processing()`; in this case, we will only display it with OpenCV (don't forget the `cv::waitKey(1);` function to tell OpenCV to stop and show your images):

```cpp
void image_processing()
{
    if (is_image_){
        cv::imshow("view", image_);
        cv::waitKey(1);
    }
}
```

### Python

Similarly, in Python we first start our node and name it:

```python
super().__init__("image_subscriber")
```

Then, we have to create a subscriber to tell ROS which function we will use when a new message comes as follows:

```python
self.subscriber_ = self.create_subscription(
    Image, "camera/image", self.callback_camera_image, 10)
```

As in the C++ example, we can process our new data inside the callback function. However, we will use the callback functions to update our variables and process them later in a custom function (e.g. our timer function). So, in Python, we declare our callback function only with the name of our message but we let the data type as an implicit value:

```python
def callback_camera_image(self, msg)
```

We then transform our ROS message to an OpenCV array:

```python
#Transform the new message to an OpenCV matrix
bridge_rgb=CvBridge()
self.img = bridge_rgb.imgmsg_to_cv2(msg,msg.encoding).copy()
```

and we let know Python that we have received our first message:

```python
self.is_img = True
```

Finally, we declare our timer using a basic structure `create_timer(time_in_seconds, callback_function)` as:

```python
self.processing_timer_ = self.create_timer(0.030, self.image_processing) #update image each 30 miliseconds
```

We, then, can process our data in the callback function; in our example, we are going to display our image:

```python
def image_processing(self):
    if self.is_img:
        #Show your images
        if(self.display):
            cv2.imshow("view", self.img)
            cv2.waitKey(1)
``` 

As you can see, our object-oriented programming (OOP) classes are similar in C++ and Python; however, we prefer C++ for performance but we also use Python for simplicity whenever high performance is not required.

## 1.3.2 Test your code

Run the following command in a terminal to compile your code:

```bash
cd ~/robovision_ros2_ws
colcon build
```

Then, run the next command:

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_images my_publisher ~/robovision_ros2_ws/src/robovision_ros2/data/images/baboon.png
```

Now, in a different terminal, run the following commands:

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_images my_subscriber
```

What did happen?

In a new terminal run this command:

```bash
ros2 node list
```

```bash
ros2 node info /image_subscriber
```

Can you get what is happening?

```bash
/image_subscriber
  Subscribers:
    /camera/image: sensor_msgs/msg/Image
```

The `ros2 node info` let us know that our *Image* topic `/camera/image` is being accessed by the node with name `/image_subscriber`. Remember that we named our image publisher as `create_publisher<sensor_msgs::msg::Image>("camera/image", 10)` and that we subscribed to the `/camera/image` topic using the declaration `create_subscription<sensor_msgs::msg::Image>("camera/image", 10, std::bind(&ImageSubscriberNode::callback_image, this, std::placeholders::_1))`.

## 1.3.3 Homework 1.3

* Create a second subscriber to subscribe to our second topic, the scaled input image, in the `/camera/scaled_image` topic, and display it.

**Hint** In C++, you need to create a second subscriber and a corresponding callback to the selected topic:

```cpp
scaled_image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
            "camera/scaled_image", 10, std::bind(&ImageSubscriberNode::callback_scaled_image, this, std::placeholders::_1));
```

and declare your new callback function *callback_scaled_image*:

```cpp
void callback_scaled_image(const sensor_msgs::msg::Image::SharedPtr msg)
```

Similarly, in Python, you need to subscribe to the new topic:

```python
self.scaled_image_subscriber_ = self.create_subscription(
    Image, "camera/scaled_image", self.callback_scaled_image, 10)
```

and define your new callback function *callback_scaled_image*:

```python
def callback_scaled_image(self, msg)
```

## Authors

* **Luis Contreras** - [ARTenshi](https://artenshi.github.io/)
* **Hiroyuki Okada** - [AIBot](http://aibot.jp/)

