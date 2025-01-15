# Image Publishers and Subscribers in ROS

The goal of this repository is to introduce students to image publishers and subscribers using ROS and OpenCV.

# 0. Get the introduction to robot vision libraries

## 0.1 Clone this repository

**Warning:** *You only need to do this once. If you have already created this repository in your local machine, pulling it again may cause a lost of your information.*

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
git clone https://gitlab.com/trcp/robointro.git
cd ..
catkin_make
```

# 1. ROS Publishers and Subcribers

We asume the students have a notions on these subjects. If it is not the case, they can start here, for C++:

> http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

and here, for Python:

> http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

# 1.1 Static image publisher

Have a look at the `my_publisher.cpp` file. This file presents the basic structure to construct a **publisher** in ROS.

## 1.1.1 Create a publisher

First, you need to create your **start your node** and give it a unique name:

```
ROS_INFO("Starting image_publisher application...");
ros::init(argc, argv, "image_publisher");
```

Then, we need to create a **ROS handler** to tell the system what we intend to do and reserve the appropiate resources. Here, we give a unique name to each of our output topics, so anyone else can access to them without confusion. In this case, we name our output topic `camera/image`, as follows:

```
ros::NodeHandle nh;
image_transport::ImageTransport it(nh);
image_transport::Publisher pub = it.advertise("camera/image", 1);

```

Now, let's gather some data. We will open an image using OpenCV and then publish it in our topic. First, we read the image:

```
cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
```

and we convert it to a **ROS message**, the type of data that can be send through the ROS framework:

```
sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
```

Now we can publish this message. 

We can do it once (`pub.publish(msg);`), but it means that the information will be only availabre a fraction of time, if you don't access to it at that very moment, you won't be able to use it anymore. So, let's publish it all the time! We first decide a frame rate, it is, the number of **frames per second** (fps). In general, we consider **real time** something around 30 fps. We use a `while` loop to publish our image at 30 Hz (i.e. 30 fps):

```
ros::Rate rate(30);
while (nh.ok())
{
	pub.publish(msg);

	//Prepare ROS to publish the next message
	ros::spinOnce();
	rate.sleep();
}
```

And that's it, we have our first ROS publisher. 

## 1.1.2 Test your code

Run the following command in a terminal:

```
roscore
```

In a second terminal, run the next command:

```
rostopic list
```

You should see something like:

```
/rosout
/rosout_agg
```

Now, in the same terminal, run the following commands:

```
source ~/tidbots_ws/devel/setup.bash
rosrun introvision_images my_publisher ~/tidbots_ws/src/robointro/1_images/data/baboon.png
```

Then, again, in a new terminal, run this command:

```
rostopic list
```

Do you remember this line `image_transport::Publisher pub = it.advertise("camera/image", 1);`? Well, now we can see our topic `/camera/image`! Furthermore, we can get the details of it. If we enter the command:

```
rostopic info /camera/image
```

we can see:

```
Type: sensor_msgs/Image

Publishers: 
 * /image_publisher

Subscribers: None

```

So, we can see that our topic is a *sensor_msgs/Image* data and that the *Publisher* correspond to the name we gave it when we started the node a few lines above `ros::init(argc, argv, "image_publisher");`. However, we don't have any *Subscriber* yet. Let's solve it in Section 1.3.

## 1.1.3 Homework 1.1

* Add a new publisher in your code that publishes a scaled version by half of the original image.

To give you an idea on how ROS works, we will help you to solve this task this time, but you are expected to solve it all by yourself.

Do you remember our ROS handler? We need one handler per topic, so let's add a new one (don't forget to give a different and unique name to each topic, in this case, we named it `camera/image_2`):

```
ros::NodeHandle nh2;
image_transport::ImageTransport it2(nh2);
image_transport::Publisher pub2 = it2.advertise("camera/image_2", 1);
```

After a short search on internet, we found that, to scale an image in OpenCV, we can use the following command:

```
cv::Mat image2;
cv::resize(image, image2, cv::Size(), 0.5, 0.5, CV_INTER_AREA);
```

Now, we need to publish our new image:

```
sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image2).toImageMsg();
```

Finally, inside the while loop we publish our new message into our second topic ar 30fps:

```
pub2.publish(msg2);
```

Your code should look something like:

```
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
	//Start your ROS node
	ROS_INFO("Starting image_publisher application...");
	ros::init(argc, argv, "image_publisher");

	//Create a handler for your ROS element
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image", 1);

	//Create a handler for your next ROS element
	ros::NodeHandle nh2;
	image_transport::ImageTransport it2(nh2);
	image_transport::Publisher pub2 = it2.advertise("camera/image_2", 1);

	//Read the input data
	cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

	//Process  your input data
	cv::Mat image2;
	cv::resize(image, image2, cv::Size(), 0.5, 0.5, CV_INTER_AREA);

	//Convert the output data into a ROS message format
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image2).toImageMsg();

	//Let's publish our images at a frequency of 30 frames per second
	ros::Rate rate(30);
	while (nh.ok())
	{
		//Publish your messages in your ROS topics
		pub.publish(msg);
		pub2.publish(msg2);

		//Prepare ROS to publish the next message
		ros::spinOnce();
		rate.sleep();
	}
}
```

Now let's test it!

First, we need to compile our code, in a new terminal run:

```
cd ~/tidbots_ws
catkin_make
```

Now, run the following command:

```
roscore
```

Then, in a different terminal, run:

```
source ~/tidbots_ws/devel/setup.bash
rosrun introvision_images my_publisher ~/tidbots_ws/src/robointro/1_images/data/baboon.png
```

Finally, in a new terminal, run this command:

```
rostopic list
```

What can you see? Please, explain.

# 1.2 Video publisher

Have a look at the `my_video_publisher.cpp` file. This file presents the basic structure to open a camera and create a **publisher** in ROS. Can you note the similarities and differences between static image and video publishers?

## 1.2.1 Create a publisher

We started our node and our ROS handler as before. Please note that we use the same name for the static image and the video publishers, so you can only use one of them at a time.

Let's open our camera! Every camera connected to your computer has an ID number, starting from 0. If you have a laptop with an extra USB camera attached to it, the laptop's camera will have ID 0 and the USC camera ID 1, and so on. In any case, provide you have at least one camera cnnected to your computer, there will be a device with ID 0 and, therefore, the following lines should open it:

```
int video_source = 0;
cv::VideoCapture cap(video_source);
if(!cap.isOpened()) return 1;
```

Now, we should notice that, unlike static images, video sequences change in time and therefore we should update our frame at a desired frame rate (depending on your device specifications). We first create a matrix to storage this information and then, in the while loop we update it. In general, a camera takes some time to start after you call it, so we need to wait until our program starts receiving a video stream (we use the `if(!frame.empty())` to do that). Ten, our code to read and publish camera images is:

```
cv::Mat frame;
sensor_msgs::ImagePtr msg;

ros::Rate loop_rate(30);
while (nh.ok())
{
	cap >> frame;

	if(!frame.empty())
	{
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		pub.publish(msg);
	}

	ros::spinOnce();
	loop_rate.sleep();
}
```

## 1.2.2 Test your code

Run the following command in a terminal:

```
roscore
```

In a second terminal, run the next command:

```
rostopic list
```

You should see somhing like:

```
/rosout
/rosout_agg
```

Now, in the same terminal, run the following commands:

```
source ~/tidbots_ws/devel/setup.bash
rosrun introvision_images my_video_publisher
```

Then, again, in a new terminal, run this command:

```
rostopic list
```

Can you explain the output? How do you get extra information from your topic?

## 1.2.3 Homework 1.2

* Add a new publisher in your code that publishes a scaled version by half of the original video frame.

**Hint** You sould start your new frame and message outside the while loop video but process them inside the loop.

# 1.3 Image subscriber

Now that we have a stream of images being published in ROS, let's do something with them. Have a look at the `my_subscriber.cpp` and `my_subscriber.py` files. The important part here are the **callback functions**. Again, review your concepts on ROS publishers and subscribers using the links provided before.

## 1.3.1 Create a subscriber

### C++

As with any node in ROS, we need to start it and give it a unique name:

```
ros::init(argc, argv, "image_listener");
```

Now, again, we need to create a ROS handler for our subcriber. Here, we tell ROS which function will be calles every time a new image arrives:

```
ros::NodeHandle nh;
image_transport::ImageTransport it(nh);
image_transport::Subscriber sub = it.subscribe("camera/image", 1, callback_image);
```

That was easy... but now we need to create the callback function! The basic structure consist of a function's name and, as a parameter, the variable with the data type that will enter. In this case, our `msg` variable is of type `ImageConstPtr`:

```
void callback_image(const sensor_msgs::ImageConstPtr& msg)
```

Inside this function, we can process our data. The `try` and `catch` statementa in C++ are to prevent our program to break if an error occurs, but can be ommited. Therefore, let's focus on our callback function. We first need to transform our `ImageConstPtr` data in the `msg` variable to `cv::Mat`:

```
cv::Mat img;
img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
```

And then, we can use it with our standard OpenCV functions (don't forget the `cv::waitKey(30);` function to tell OpenCV to stop and show your images):

```
if (display)
{
	cv::imshow("view", img);
	cv::waitKey(30);
}
```

### Python

Similarly, in Python we first start our node and name it:

```
rospy.init_node('image_listener', anonymous=True)
```

Then, we have to create a subscriber to tell ROS which function we will use when a new message comes. However, unlike in C++, we do not need to declare a ROS handler in Python, so we create our subscriber as follows:

```
rospy.Subscriber("camera/image", Image , callback_image)
```

In the C++ example, we processed our new data inside the callback function. Intead, the Python example, we will use the callback to update a global variable that can be used later by any function in the scope. You can use any approach in both C++ and Python depending on the task at hand.

So, in Python we declare our callback function only with the name of our message but we let the data type as an implicit value:

```
def callback_image(msg)
```

We then transform our ROS message to an OpenCV array:

```
bridge_rgb=CvBridge()
img = bridge_rgb.imgmsg_to_cv2(msg,msg.encoding).copy()
```

and we let know Python that we have received out first message:

```
is_img = True
```

Finally, we use our image as a standard OpenCV variable inside our main function (do you remember why do we use a frequency of 30 Hz in the `rospy.Rate(30)` declaration?):

```
loop=rospy.Rate(30)
while not rospy.is_shutdown():
	if is_img:
		if(display):
			cv2.imshow("view", img)
			cv2.waitKey(1)

		loop.sleep()
``` 

## 1.3.2 Test your code

Run the following command in a terminal:

```
roscore
```

In a second terminal, run the next command:

```
source ~/tidbots_ws/devel/setup.bash
rosrun introvision_images my_publisher ~/tidbots_ws/src/robointro/1_images/data/baboon.png
```

Now, in a different terminal, run the following commands:

```
source ~/tidbots_ws/devel/setup.bash
rosrun introvision_images my_subscriber
```

What did happen?

In a new terminal run this command:

```
rostopic info /camera/image
```

Now the output changed! Can you spot the difference?

```
type: sensor_msgs/Image

Publishers: 
 * /image_publisher

Subscribers: 
 * /image_listener
```

The `rostopic info` let us know that our *Image* topic `/camera/image` is being accessed by the node with name `/image_listener`... that's our node! Remember that we named our node `ros::init(argc, argv, "image_listener");` and that we subscribed to the `/camera/image` topic using the declaration `image_transport::Subscriber sub = it.subscribe("camera/image", 1, callback_image);`.

## 1.3.3 Homework 1.3

* Create a second handler that subscribes to our second topic, the scaled input image, in the `/camera/image_2` topic, and display it.

**Hint** In C++, you need to create a second ROS handler to subscribe to the selected topic:

```
ros::NodeHandle nh2;
image_transport::ImageTransport it2(nh2);
image_transport::Subscriber sub2 = it2.subscribe("camera/image_2", 1, callback_image_2);
```

and declare your new callback function *callback_image_2*:

```
void callback_image_2(const sensor_msgs::ImageConstPtr& msg)
```

Similarly, in Python you need to subscribe to the new topic:

```
rospy.Subscriber("camera/image_2", Image , callback_image_2)
```

and define your new callback function *callback_image_2*:

```
def callback_image_2(msg):
```

In Python, don't forget to declare your new global variables!

## Authors

* **Luis Contreras** - [AIBot](http://aibot.jp/)
* **Hiroyuki Okada** - [AIBot](http://aibot.jp/)

