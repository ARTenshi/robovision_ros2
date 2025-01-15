#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import traceback


class ImageSubscriberNode(Node):
    def __init__(self):
        super().__init__("image_subscriber")

        self.img = []
        self.is_img = False
        self.display = True

        #Subscribers
        self.subscriber_ = self.create_subscription(
            Image, "camera/image", self.callback_camera_image, 10)
        
        #Processing
        self.processing_timer_ = self.create_timer(0.030, self.image_processing) #update image each 30 miliseconds

        self.get_logger().info("Starting image_subscriber application in python...")

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
            #Show your images
            if(self.display):
                cv2.imshow("view", self.img)
                cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
