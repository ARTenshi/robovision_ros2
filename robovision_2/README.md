# RGB Image Processing with OpenCV and ROS

The goal of this repository is to introduce students to RGB image processing using OpenCV and ROS.


# 1. Getting to know your image

Before starting to manipulate images, we should understand what they are and how we can access their data. In this work, we understand the images as a 2D array, or matrix, where each element (also known as **pixel**) in the array has a color value. We use three color channels per element in the array: Red, Green, and Blue. The origin of this image matrix is at the top-left corner and column values increase positively from left to right while row values increase positively from top to bottom, as can be seen in the image below:

<p align="center">
  <img src="images/digital_image.jpg" width="800">
</p>

Each color channel has an integer value between 0 and 255. For example, a value of RGB = [255, 0, 0] represents the red color because the value red = 255 is the maximum possible while the green = 0 and blue = 0 values represent the absence of those color. Similarly, RGB = [0, 255, 0] represents the green color and RGB = [0, 0, 255] represents the blue color. The combination of these three channels with different intensities gives us our perception of true color (for example, if you combine different values of red and green you will obtain a range of tonalities of yellow, e.g. RGB = [128, 128, 0]).


# 2. Basic operations

From our previous tutorial, we learnt how to subscribe to a ROS Image topic in C++:

```cpp
image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/image", 10, std::bind(&ImageProcessingNode::callback_image, this, std::placeholders::_1));
```

and to transform our ROS Image message to an OpenCV matrix array:

```cpp
image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
```

In Python, to create an Image subscribe:

```python
self.subscriber_ = self.create_subscription(
    Image, "camera/image", self.callback_camera_image, 10)
```

and to convert it to a matrix array:

```python
bridge_rgb=CvBridge()
self.img = bridge_rgb.imgmsg_to_cv2(msg,msg.encoding).copy()
```

## 2.1 Image dimensions

### C++

Have a look at the `my_processing.cpp` file. Let's inspect our image. First, let's see the dimensions of our image (we add a `counter_` index so we print get these values only in the first iteration):

```cpp
if (counter_ == 0)
    std::cout << "size: rows: " << image_.rows << 
                 ", cols: " << image_.cols << 
                 ", depth: " << image_.channels() << std::endl;
```

Let's test our code. First, we need to compile our code:

```bash
cd ~/robovision_ros2_ws
colcon build
```

Now, as in our previous examples, run:

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_images my_publisher ~/robovision_ros2_ws/src/robovision_ros2/data/images/baboon.png
```

Finally, in a new terminal, run this command:

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_processing my_processing
```

where we should be able to see information regarding our image's size.

### Python

Have a look at the `my_processing.py` file. Similarly, we first determine the dimensions of our image. In Python, we use the `shape` operator in our matrices, it returns three values (the number of rows, columns and channels) for color images and two values (rows and columns) for grayscale images. So you can use the length of this vector to determine if your image is a multi- or single-channel array.

```python
if (self.counter == 0):
    (rows,cols,channels) = self.img.shape #Check the size of your image
    print ('size: rows: {}, cols: {}, channels: {}'.format(rows, cols, channels))
```

Now, let's try our code. In a pure Python project, we can build our code using the `--symlink-install` flag once and test modifications of our file without the need to build it again; however, in this project, we mix C++ and Python code, so we need to build it every time we make any modification. So, run the following command:

```bash
cd ~/robovision_ros2_ws
colcon build
```

Teen, we run:

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_images my_publisher ~/robovision_ros2_ws/src/robovision_ros2/data/images/baboon.png
```

Finally, in a new terminal:

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_processing my_processing.py
```

where we should be able to see information regarding to our image.

### Homework 2.1

* What is the size (columns, rows, and channels) of our image?

## 2.2 Image manipulation

To understand how to manipulate our images, let's first make some easy edits. In this case, we will add an increasing id number to our image. Remember that the top-left corner is our image origin (0,0), and that the columns increase from left to right and the rows from top to bottom. 

### C++

To add a text into our image `img`, we use the OpenCV function `cv::putText`. We have an integer `counter` variable that we increase each time we receive and process a new image, and we transform it into a text sequence using the `std::to_string(counter)` function.

To understand how an image is coded in OpenCV, play around with the `cv::Point` and `CV_RGB` values. The cv::Point(col_id, row_id) marks the origin of our text (i.e. the bottom-left corner of our text box); use different values of col_id and row_id and see what happens.

On the other hand, the CV_RGB(red, green, blue) parameter determines our text's color. Remember that a single color channel ranges from 0 to 255, so try different red, green, and blue combinations and see all the colors you can create!

```cpp
cv::putText(image_,
	std::to_string(counter_),
	cv::Point(25, 25), //change these values cv::Point(col_id, row_id)
	cv::FONT_HERSHEY_DUPLEX,
	1.0,
	CV_RGB(255, 0, 0), //change these values CV_RGB(red, green, blue)
	2);
```

Compile and test your code as in the previous section.

### Python

Likewise, we use the `cv2.putText` function to add a text string into our `img` array. The two dimensional vector (col_id, row_id) marks the origin (bottom-left) of our text box. The three dimensional vector indicates our text's color. However, in Python **the order of our color channels is (blue, green, red)**. Be aware of this difference when you work with C++ and Python at the same time. Now, give different values to the color vector and see how to behave.

```python
cv2.putText(self.img, 
	str(self.counter),
	(25,25), 
	cv2.FONT_HERSHEY_SIMPLEX, 
	1, 
	(255, 0, 0), 
	2, 
	cv2.LINE_AA)
```

Again, try your code as in the previous section.

You should see something like

<p align="center">
  <img src="images/baboon_number.jpg" width="500">
</p>

### Homework 2.2

* Provide the output image for five different text positions and color combinations.

## 2.3 Image transformations

## 2.3.1 BGR to Grayscale

Now that we have an idea of the composition of an image and an understanding of our code, let's add some image transformations. We will show you how to apply one of the built-in functions in OpenCV, so you can explore all the available functions depending on your task at hand.

Let's change our image from color to grayscale. We can understand a grayscale as a color image where the `Gray` value is a combination of the different color channels as follows:

![equation](https://latex.codecogs.com/svg.latex?Gray=((0.3*R)&plus;(0.59*G)&plus;(0.11*B)))

and it has the same value in its three channels, i.e. RGB = [Gray, Gray, Gray]. Therefore, to reduce the memory used, many systems only use a one-channel matrix with a single Gray value per pixel: RGB = [Gray]. 

### C++

Please complete the provided code with the following instructions as appropriate. We first create a new matrix `image_gray` and apply the OpenCV `cvtColor` function to our original `image_` image:

```cpp
cv::Mat image_gray;
cv::cvtColor(image_, image_gray, CV_BGR2GRAY);
```

Let's inspect our image:

```cpp
if (counter_ == 0)
    std::cout << "image_gray size: rows: " << image_gray.rows << 
                 ", cols: " << image_gray.cols << 
                 ", depth: " << image_gray.channels() << std::endl;
```

and display it in a new window:

```cpp
cv::imshow("gray", image_gray);
```
Compile and test your code as in the previous sections.

### Python

Similarly, we apply our `cvtColor` function to our original `img` and store it in a new `img_gray` array.

```python
img_gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
```

Then, we inspect our image. Remember that our `shape` operator returns three values for color images and two values for grayscale images and that you can use the length of this vector to determine if your image is a multi- or single-channel array.

```python
if (self.counter == 0):
    (rows,cols) = img_gray.shape
    print ('img_gray size: rows: {}, cols: {}'.format(rows, cols))
    print ('length (img_gray): {}'.format(len(img_gray.shape)))
```

and show it:

```python
cv2.imshow("gray", img_gray)
```

Now, save your files and test them as before. What can you see? Did you notice how a grayscale image uses only a single channel to represent all the information instead of three channels with the same value?

Again, try your code as in the previous section.

The expected output is

<p align="center">
  <img src="images/baboon_gray.jpg" width="500">
</p>

### Homework 2.3.1

* What is the size (columns, rows, and channels) of our grayscale image? Show the resulting image.

* What is the scope of your grayscale image variables? Do you need to make them global? Why?

## 2.3.2 Color Thresholding

Now we will create a very basic color segmentator. For simplicity, we will use the BGR color space, but it is more common to use the HSV color space; we let the interested reader to find what are the extra steps to use an HSV image for color segmentation.

An easy way to create a color-based segmentator is to find all the pixels that are around our target color. For example, if we want to segment a high-intensity red color with a red channel value around `my_red=[225]`, we need to find all the pixels in the range (my_red-delta , my_red+delta) where `delta` is a value that we define; it is important to note that, in this example, the other channels range from 0 to 255.

Therefore, we have two steps to create a segmented image. First, we find the pixels inside our desired range and mark them as ONE and all the pixels outside the range as ZERO, we call this array with ZEROs and ONEs a **mask**. Then, we create an output image where all the elements in the MASK have a color value from the input image and all other pixels are zero (or black).

### C++

First, we create our mask image (don't forget to declare it!) and fill it with zeros and ones, depending on whether the pixel is inside or outside our color range (**remember the order in our array: [Blue,Green,Red]**), respectively:

```cpp
cv::Mat mask;
cv::inRange(image_, cv::Scalar(0,0,220), cv::Scalar(240,255,240), mask);
```
In the example, our target color is `red=255`, and `delta=20`, the blue and green channels vary from 0 to 255. Then, we copy only those pixels that met our range condition. We use the `image_.copyTo(output,mask)` to copy to our *output* image only those pixels with a ONE in our *mask*; we declare our output as `image_filtered_` and then:

```cpp
cv::Mat image_filtered;
image_.copyTo(image_filtered, mask);
```

Finally, don't forget to show your images

```cpp
cv::imshow("mask", mask);
cv::imshow("image_filtered", image_filtered);
```

Compile and test your code as in the previous sections.

A segmented image in the red channel should look like

<p align="center">
  <img src="images/baboon_mask_red.jpg" width="800">
</p>

### Python

Here, we define the valid color range we want to use (**remember the order in our array: [Blue,Green,Red]**). Now, we will select a blue color, so `blue=220` and `delta=10`, the red and green channels vary from 0 to 255, as follow:

```python
lower_val = np.array([200,0,0])
upper_val = np.array([240,255,255])
```

Then, we create our mask with ONE value whenever a pixel is in that color range, we use the `cv2.inRange` function:

```python
mask = cv2.inRange(self.img, lower_val, upper_val)
```

Finally, we copy those pixels from our original to our segmented image as follows:

```python
image_filtered = cv2.bitwise_and(self.img,self.img, mask= mask)
```

Now, we show our results:

```python
cv2.imshow("mask", mask)
cv2.imshow("image_filtered", image_filtered)
```

Again, try your code as in the previous section.


A segmented image in the blue channel should look like

<p align="center">
  <img src="images/baboon_mask_blue.jpg" width="800">
</p>

### Homework 2.3.2

* Create five segmented images using different color ranges. From the previous unit, do you remember how to start an image publisher using your camera? Try using it and place objects with different colors in front of your camera and see what happens!


# 3. Per-element operations

Although OpenCV comes with a variety of functions, we will always need to access the elements of our matrix for a number of reasons (from simple inspection to the implementation of our own algorithms). Here, we will provide a simple way to access all the elements in your array.

## 3.1 Single element access

Let's first inspect one pixel value of our array at a given (row_id, col_id) position. **Remember that our array starts in `(0,0)` and therefore the last element in our image `image_` (at the bottom-right corner) is `(image_.rows - 1, image_.cols - 1)`**. 

### C++

We use the `image_.at<cv::Vec3b>(row_id,col_id)` attribute in our `image_` matrix to get a color element in the [Blue,Green,Red] order, and `image_.at<uchar>(row_id,col_id)` to get a grayscale value -- if you don't know what is a `uchar` data type, please review that concept; in short, it is an integer that goes from 0 to 255. **Notice the id input order in our function, the first index ALWAY corresponds to the rows and the second index to the columns**; be careful with the elements' order when you use OpenCV functions, as an example, remember that in our `cv::putText` function the `cv::Point` element has a (col_id, row_id) order while the `at<>` attribute of our image has a (row_id,col_id) order. 

In this case, we will inspect the middle point in our array as follows:

```cpp
if (counter == 0)
{
	int row_id, col_id;
	row_id = image_.rows/2;
	col_id = image_.cols/2;

        std::cout << "pixel value in img at row=" << row_id <<
                     ", col=" << col_id <<
                     " is: " << image_.at<cv::Vec3b>(row_id,col_id) << std::endl;
        std::cout << "pixel value in img_gray at row=" << row_id <<
                     ", col=" << col_id <<
                     " is: " << (int)image_gray.at<uchar>(row_id,col_id) << std::endl;
}
```

Compile and test your code as in the previous sections.

**Important: You should notice that, for color images, the output of the `img.at<cv::Vec3b>(row_id,col_id)` operator is a 3D vector with the [Blue,Green,Red] information, in that order.** 

### Python

Here, we access the elements in our `img` array as in any other Python array: `img[row_id,col_id]` to get a color pixel in [Blue,Green,Red] order or a grayvalue scalar. **We ALWAYS indicate the rows first and then the columns**; again, be careful with the input order when you use different OpenCV functions in Python. Then, we get the values of our middle pixel element as:

```python
if (counter == 0):
    (rows,cols,channels) = self.img.shape
    row_id = int(rows/2)
    col_id = int(cols/2)

    print ('pixel value in img at row: {} and col: {} is {}'.format(row_id, col_id, self.img[row_id,col_id]))
    print ('pixel value in img_gray at row: {} and col: {} is {}'.format(row_id, col_id, img_gray[row_id,col_id]))
```

Again, try your code as in the previous section.

**IMPORTANT: You should notice that, for color images, the output of the `img[row_id,col_id]` operator is a 3D vector with the [Blue,Green,Red] information, in that order.** 

### Homework 3.1 

* Provide the values of ten different pixels in your image.

Do you remember the equation to obtain the Gray value from a combination of the different color channels? Does it apply to your image?


## 3.2 Multi-element access

Now that we know how to access an element in our image and the kind of information in it (color or grayscale), let's access all our pixels consistently. Remember that the Gray value is a combination of the three different color channels in an image. A common function that approximates the gray value is as follows:

![equation](https://latex.codecogs.com/svg.latex?Gray=((0.3*R)&plus;(0.59*G)&plus;(0.11*B))) 

so let's apply that equation to all and every pixel in our image. To do so, we need to run a *nested for loop* where one index goes from the first to the last column and the other index goes from the first row to the last. Remember that, in general, **the array indices start in zero and, therefore, they should end at (width-1) and (height-1)**.

### C++

First, we need to create a single channel matrix to store our new image, so it should have the same dimensions as our input and each value should be `uchar`:

```cpp
cv::Mat image_gray_2 = cv::Mat::zeros(image_.rows,image_.cols, CV_8UC1);
```

Then, we create our indices to access all the elements in our image. Please, remember which index corresponds to the rows and what to the columns! In our case, the `i` variable corresponds to the rows and the `j` index to the columns:

```cpp
for(int i=0; i<image_.rows; i++)
	for(int j=0; j<image_.cols; j++)
```
Now, we access the pixel value of all (i,j) pixels and create a Gray value from the combination of the color channels.  We use the `image_.at<cv::Vec3b>(row_id,col_id)` operator; remember that this operator's output is a [Blue,Green,Red] vector so, we have:

```cpp
int gray_val = 0.11*image_.at<cv::Vec3b>(i,j)[0] + 0.59*image_.at<cv::Vec3b>(i,j)[1] + 0.3*image_.at<cv::Vec3b>(i,j)[2];
```

We store the `int` grayscale value in our `uchar` new `uchar` matrix at the (i,j) position:

```cpp
image_gray_2.at<uchar>(i,j) = (unsigned char)gray_val;
```

Your code should look something like:

```cpp
cv::Mat image_gray_2 = cv::Mat::zeros(image_.rows,image_.cols, CV_8UC1);
for(int i=0; i<image_.rows; i++)
    for(int j=0; j<image_.cols; j++)
    {
        int gray_val = 0.11*image_.at<cv::Vec3b>(i,j)[0] + 0.59*image_.at<cv::Vec3b>(i,j)[1] + 0.3*image_.at<cv::Vec3b>(i,j)[2];
        image_gray_2.at<uchar>(i,j) = (unsigned char)gray_val;
    }
```

Finally, don't forget to display your image (each new window should have its unique name):

```cpp
cv::imshow("gray_2", image_gray_2);
```

Compile and test your code as in the previous sections.

### Python

Similarly, we start by defining our new `uint8` matrix (the `uint8` data type in Python is similar to the `uchar` data type in C++):

```python
(rows,cols,channels) = img.shape
img_gray_2 = np.zeros( (rows,cols,1), np.uint8 )
```

Then, we create our indices (please refer to the manual to know how the `range(int)` function works; basically, it creates a sequence of numbers from 0 to int-1), remember which index corresponds to the rows and the columns:

```python
for i in range(rows):
	for j in range(cols):
```

Then, access to the information for pixel (i,j), combine it to create a grayscale value, and store it in our new grayscale image:

```python
for i in range(rows):
	for j in range(cols):
```

Your code should look like:

```python
(rows,cols,channels) = self.img.shape
img_gray_2 = np.zeros( (rows,cols,1), np.uint8 )

for i in range(rows):
    for j in range(cols):
        p = self.img[i,j]
        img_gray_2[i,j] = int( int(0.11*p[0]) + 0.59*int(p[1]) + int(0.3*p[2]) )
```

Finally, don't forget to show your new image:

```python
cv2.imshow("gray_2", img_gray_2)
```

Again, try your code as in the previous section.

**IMPORTANT: Can you notice the difference in velocity!? Although C++ seems more cumbersome than Python, the execution speed is much faster. A common practice is to use C++ for the core operations in our applications and Python for the high-level processes.**

### Homework 3.2

* Create a loop to access all the elements in your grayscale image and rotate it 180 degrees. The result should be as in the image below.

<p align="center">
  <img src="images/baboon_rotated.jpg" width="300">
</p>

**Hint** You should create a new matrix before starting your loop. Also, observe the indices in your original and your new image. what is their relationship?

## Authors

* **Luis Contreras** - [ARTenshi](https://artenshi.github.io/)
* **Hiroyuki Okada** - [AIBot](http://aibot.jp/)

