# Services and Clients in ROS

In ROS2, clients and services enable synchronous communication between nodes. Unlike publishers and subscribers, which facilitate continuous data streams, clients and services are designed for request-response interactions. A client node sends a request to a service, and the service node processes the request and sends back a response. This is ideal for tasks that require specific actions or immediate feedback, such as controlling a robot arm or querying a sensor's state.

You can check some basic concepts for C++:

> https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html

and for Python:

> https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html

# 1. ROS2 Interfaces

In ROS2, interfaces allow nodes to communicate using predefined data structures. Interfaces come in two forms:
- **Messages (`msg`)**: Define the structure of data for topics.
- **Services (`srv`)**: Define request-response interactions for services.

This tutorial uses examples from the `robovision_interfaces` package to demonstrate creating and using ROS2 interfaces.

---

## Setting Up the Package

Organize the folder structure for your custom interfaces as follows:

```
robovision_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   └── ObjectCentroid.msg
└── srv/
    └── GetPointCenter.srv
```

This folder is at the same level as any new ROS2 package in your project.

## Defining a Custom Message: `ObjectCentroid.msg`

A custom message describes the data structure for topics. The `ObjectCentroid.msg` defines the centroid coordinates and an array:

```
float64   x
float64   y
float64   z
float64[] centroid
```

where 

- **`float64 x, y, z`**: Represent the 3D coordinates of the centroid.
- **`float64[] centroid`**: A dynamic array to store additional data points.

## Defining a Custom Service: `GetPointCenter.srv`

A custom service defines the structure of a request and a response. The `GetPointCenter.srv` file looks like this:

```
int64          x
int64          y
---
ObjectCentroid point
```

where

- **Request (`int64 x, y`)**: Accepts two integer inputs (e.g., pixel coordinates).
- **Response (`ObjectCentroid point`)**: Returns the computed centroid as an `ObjectCentroid` message.

The `---` separates the request and response parts of the service definition. Notice that, if the message is defined in the same package, the package name does not appear in the service or message definition. If the message is defined elsewhere, we have to specify it, e.g. `robovision_interfaces/msg/ObjectCentroid point`.

## Integrating the Interfaces into the Build System

Update the `CMakeLists.txt` to include the message and service definitions:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectCentroid.msg"
  "srv/GetPointCenter.srv"
)

ament_export_dependencies(rosidl_default_runtime)
```

and update the `package.xml` to declare dependencies:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
```

# 2. ROS Service

The main difference between a topic and a service is that, while a topic is working, a service works under request (that might save resources).

Let's compare our "rgbd_reader" and our "robovision_service" files (both in C++ and Python). They are very similar! We have two main changes. First, we don't have a publisher, as it sends a response under request. Second, we don't have a timer, as it is not working indefinitely. Instead, we create a ROS2 service that enters a callback function when we call the service. In Python it is

```python
self.get_point_center_service_ = self.create_service(
    GetPointCenter, "get_point_center", self.point_cloud_processing)
```

and in C++

```cpp
get_point_center_service_ = this->create_service<robovision_interfaces::srv::GetPointCenter>(
    "get_point_center", 
    std::bind(&PointCloudCentroidNode::point_cloud_processing, this, 
	      std::placeholders::_1, std::placeholders::_2));
```

Notice that we need to declare the service type, in this case, we are using our custom interface `robovision_interfaces/srv/GetPointCenter`. Also, we changed the definition of our callback function to incorporate the `request` and `response`. Finally, also note that we need to fill our `response` and return it (instead of publishing the result in a custom topic).

## 2.1 Test your code

First, let's compile it

```bash
cd ~/robovision_ros2_ws
colcon build
```

and start it (don't forget to start your rosbag!)

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_services robovision_service
```

In a different terminal, enter

```bash
ros2 service list
```

Can you see the service we just declared? Now enter

```bash
ros2 service type /get_point_center
```

What's the interface it is using? You can inspect it

```bash
ros2 interfaces show robovision_interfaces/srv/GetPointCenter
```

Finally, let's call our service

```bash
ros2 service call /get_point_center robovision_interfaces/srv/GetPointCenter "{x: 320, y: 240}"
```

What was the result?

# 3. ROS Client

In ROS2, clients are used to interact with services in a synchronous request-response manner. This tutorial will demonstrate how to implement and utilize a ROS2 client. The example involves a `PointCloudCentroidNode` that calls a service named `get_point_center` to compute the centroid of a 2D point.

## 3.1 Client in Python

The `robovision_client.py` file demonstrates a ROS2 client implementation for interacting with the `GetPointCenter` service. Below, we break down its key components.

### 3.1.1 **Node Initialization**

The `PointCloudCentroidNode` class initializes a ROS2 node named `point_cloud_client`.

```python
self.declare_parameter("x", 320)
self.declare_parameter("y", 240)

self.x_ = self.get_parameter("x").value
self.y_ = self.get_parameter("y").value
```
Parameters `x` and `y` define the target coordinates to query from the service.

### 3.1.2 **Service Call Setup**

To show that we can call a service at any moment, we create a ROS2 timer to call the service periodically every 2.5 seconds:

```python
self.client_call_timer_ = self.create_timer(2.5, self.client_caller)
```

The `client_caller()` function prepares and sends a service request through a `call_get_point_center_server`:

```python
def client_caller(self):
    self.call_get_point_center_server(self.x_, self.y_)
```

Every time we want to call our service, we use this function so, you can use it as a template. We will explain it next.

#### **Creating the Client and Making a Request**

A ROS2 client is created with the specified service type (`GetPointCenter`) and name (`get_point_center`):

```python
client = self.create_client(GetPointCenter, "get_point_center")
while not client.wait_for_service(1.0):
    self.get_logger().warn("Waiting for get_point_center service...")
```

The request message is populated, and an asynchronous call is made:

```python
request = GetPointCenter.Request()
request.x = _x
request.y = _y

future = client.call_async(request)
future.add_done_callback(
    partial(self.callback_call_point_cloud, _x=request.x, _y=request.y))
```

#### **Handling the Response**

The callback `callback_call_point_cloud()` processes the service response (we store the response in a global variable for future use):

```python
def callback_call_point_cloud(self, future, _x, _y):
    try:
        response = future.result()
        self.get_logger().info(
            "(" + str(_x) + ", " + str(_y) + ") position is: " + str(response.point))

        self.point_ = response.point
    except Exception as e:
        self.get_logger().error("Service call failed %r" % (e,))
```

---

### 3.1.3 Test your code

First, let's compile it

```bash
cd ~/robovision_ros2_ws
colcon build
```

and start our service (don't forget to start your rosbag!)

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_services robovision_service.py
```

In a different terminal, enter

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_services robovision_client.py
```

What was the result?


## 3.2 Client in C++

The `robovision_client.cpp` file provides an equivalent implementation of the ROS2 client in C++. Below are the highlights and key differences from the Python example.

As in our Python implementation, we create a timer to show that we can call our service at any moment:

```cpp
client_call_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(2500),
    std::bind(&PointCloudCentroidNode::client_caller, this));
```

The client_caller function calls a `get_point_center()` function; we use this structure as a template. A major difference with Python is that we need to call our serves in a thread, so the program flow can continue:

```cpp
threads_.push_back(std::thread(std::bind(&PointCloudCentroidNode::call_get_point_center_server, this, x, y)));
```

The `call_get_point_center_server` in our thread works as in our Python implementation. A C++ client is created with the service type `robovision_interfaces::srv::GetPointCenter` and name `get_point_center`:

```cpp
auto client = this->create_client<robovision_interfaces::srv::GetPointCenter>("get_point_center");
while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for get_point_center service...");
}
```

The request is constructed and sent asynchronously:

```cpp
auto request = std::make_shared<robovision_interfaces::srv::GetPointCenter::Request>();
request->x = x;
request->y = y;

auto future = client->async_send_request(request);
```

The response from the service is processed synchronously by waiting for the future object:

```cpp
try {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(),
                "(%d, %d) position is: [%f, %f, %f]",
                x, y, response->point.centroid[0],
                response->point.centroid[1],
                response->point.centroid[2]);

    point_ = response->point.centroid;
} catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Service call failed.");
}
```

### 3.2.1 Test your code

First, let's compile it

```bash
cd ~/robovision_ros2_ws
colcon build
```

and start our service (don't forget to start your rosbag!)

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_services robovision_service
```

In a different terminal, enter

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_services robovision_client
```

What was the result?

## Authors

* **Luis Contreras** - [ARTenshi](https://artenshi.github.io/)
* **Hiroyuki Okada** - [AIBot](http://aibot.jp/)

