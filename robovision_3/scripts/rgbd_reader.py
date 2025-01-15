#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import cv2
from cv_bridge import CvBridge

import numpy as np
import traceback


class PointCloudCentroidNode(Node):
    def __init__(self):
        super().__init__("point_cloud_centroid")

        self.rgb_ = []
        self.depth_ = []
        self.depth_mat_ = []
        self.point_cloud_ = []

        self.centroid_=Pose()

        self.is_ptcld_ = False
        self.display_ = True

        #Publishers
        self.centroid_publisher_ = self.create_publisher(
            Pose, "/object_centroid", 10)

        #Subscribers
        self.rgb_subscriber_ = self.create_subscription(
            Image, "/camera/rgb/image_rect_color", self.callback_rgb_rect, 10)
        self.depth_subscriber_ = self.create_subscription(
            Image, "/camera/depth_registered/image", self.callback_depth_rect, 10)
        self.point_cloud_subscriber_ = self.create_subscription(
            PointCloud2, "/camera/depth_registered/points", self.callback_point_cloud, 10)
        
        #Processing
        self.processing_timer_ = self.create_timer(0.030, self.point_cloud_processing) #update image each 30 miliseconds

        self.get_logger().info("Starting point_cloud_centroid application in python...")

    def callback_rgb_rect(self, msg):
        try:
            #Transform the new message to an OpenCV matrix
            bridge_rgb=CvBridge()
            self.rgb_ = bridge_rgb.imgmsg_to_cv2(msg,msg.encoding).copy()
            self.rgb_ = cv2.cvtColor(self.rgb_, cv2.COLOR_RGB2BGR)

        except:
            self.get_logger().error(traceback.format_exc())

    def callback_depth_rect(self, msg):
        try:
            #Transform the new message to an OpenCV matrix
            bridge_depth=CvBridge()
            self.depth_=bridge_depth.imgmsg_to_cv2(msg,"32FC1").copy()

            self.depth_mat_ = np.array(self.depth_, dtype=np.float32)
            cv2.normalize(self.depth_mat_, self.depth_, 0, 1, cv2.NORM_MINMAX)

        except:
            self.get_logger().error(traceback.format_exc())

    def callback_point_cloud(self, msg):
        try:
            self.is_ptcld_ = False
            # Read the point cloud as a list of tuples (x, y, z, ...)
            self.point_cloud_ = np.array(list(pc2.read_points(msg, field_names=None, skip_nans=False)))

            # Reshape if the point cloud is organized (msg.height > 1)
            if msg.height > 1:
                self.point_cloud_ = self.point_cloud_.reshape((msg.height, msg.width, -1))
                
                rows, cols, _ = self.point_cloud_.shape
                print ('new message has arrived; point cloud size: rows: {}, cols: {}'.format(rows, cols))

                self.is_ptcld_ = True
        except:
            self.get_logger().error(traceback.format_exc())
            self.is_ptcld_ = False

    def point_cloud_processing(self):
        if self.is_ptcld_:

            #Access a point in the point_cloud
            rows, cols, _= self.point_cloud_.shape
            row_id = int(rows/2)
            col_id = int(cols/2)

            p = [float(self.point_cloud_[row_id, col_id, 0][0]), 
                 float(self.point_cloud_[row_id, col_id, 0][1]), 
                 float(self.point_cloud_[row_id, col_id, 0][2])]

            self.centroid_.position.x = p[0]
            self.centroid_.position.y = p[1]
            self.centroid_.position.z = p[2]

            self.centroid_.orientation.x=0.0
            self.centroid_.orientation.y=0.0
            self.centroid_.orientation.z=0.0
            self.centroid_.orientation.w=1.0

            print ('central point: {}'.format(p))

            #Publish centroid pose
            self.centroid_publisher_.publish(self.centroid_)

            #Show your images
            if(self.display_):
                cv2.imshow("rgb", self.rgb_)
                cv2.imshow("depth", self.depth_)
                cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudCentroidNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
