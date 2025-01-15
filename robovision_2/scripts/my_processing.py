#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import traceback


class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__("image_processing")

        self.img = []
        self.is_img = False
        self.display = True

        self.counter = 0

        #Subscribers
        self.subscriber_ = self.create_subscription(
            Image, "camera/image", self.callback_camera_image, 10)
        
        #Processing
        self.processing_timer_ = self.create_timer(0.030, self.image_processing) #update image each 30 miliseconds

        self.get_logger().info("Starting image_processing application in python...")

    def callback_camera_image(self, msg):
        try:
            #Transform the new message to an OpenCV matrix
            bridge_rgb=CvBridge()
            self.img = bridge_rgb.imgmsg_to_cv2(msg,msg.encoding).copy()
            self.is_img = True

        except:
            self.get_logger().error(traceback.format_exc())

    def image_processing(self):
        if self.is_img:

            if (self.counter == 0):
                (rows,cols,channels) = self.img.shape #Check the size of your image
                print ('size: rows: {}, cols: {}, channels: {}'.format(rows, cols, channels))

            self.counter += 1
            #Show your images
            if(self.display):
                cv2.imshow("view", self.img)
                cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
