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

from robovision_interfaces.msg import ObjectCentroid
from robovision_interfaces.srv import GetPointCenter

class PointCloudCentroidNode(Node):
    def __init__(self):
        super().__init__("point_cloud_service")

        self.rgb_ = []
        self.depth_ = []
        self.depth_mat_ = []
        self.point_cloud_ = []

        self.centroid_=Pose()

        self.is_ptcld_ = False
        self.display_ = True

        #Publishers
        #self.centroid_publisher_ = self.create_publisher(
        #    Pose, "/object_centroid", 10)

        #Subscribers
        self.subscriber_ = self.create_subscription(
            Image, "/camera/rgb/image_rect_color", self.callback_rgb_rect, 10)
        self.subscriber_ = self.create_subscription(
            Image, "/camera/depth_registered/image", self.callback_depth_rect, 10)
        self.subscriber_ = self.create_subscription(
            PointCloud2, "/camera/depth_registered/points", self.callback_point_cloud, 10)
        
        #Processing
        #self.processing_timer_ = self.create_timer(0.030, self.point_cloud_processing) #update image each 30 miliseconds

        #Services
        self.get_point_center_service_ = self.create_service(
            GetPointCenter, "get_point_center", self.point_cloud_processing)
        
        self.get_logger().info("Starting point_cloud_service application in python...")

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
            # Read the point cloud as a list of tuples (x, y, z, ...)
            self.point_cloud_ = np.array(list(pc2.read_points(msg, field_names=None, skip_nans=False)))

            # Reshape if the point cloud is organized (msg.height > 1)
            if msg.height > 1:
                self.point_cloud_ = self.point_cloud_.reshape((msg.height, msg.width, -1))

            rows, cols, _ = self.point_cloud_.shape
            self.get_logger().info('Point cloud received with size: rows: {}, cols: {}'.format(rows, cols))

            self.is_ptcld_ = True
        except:
            self.get_logger().error(traceback.format_exc())

    def point_cloud_processing(self, request, response):
        
        response.point = ObjectCentroid()

        response.point.x = 0.0
        response.point.y = 0.0
        response.point.z = 0.0
        response.point.centroid = [0.0, 0.0, 0.0]

        if self.is_ptcld_:
            if request.x and request.y:
                #Access a point in the point_cloud
                col_id = request.x
                row_id = request.y

                p = [float(self.point_cloud_[row_id, col_id, 0][0]), 
                    float(self.point_cloud_[row_id, col_id, 0][1]), 
                    float(self.point_cloud_[row_id, col_id, 0][2])]

                response.point.x = p[0]
                response.point.y = p[1]
                response.point.z = p[2]
                response.point.centroid = p


                self.get_logger().info("Centroid at ({}, {}): [{}, {}, {}]".format(row_id, col_id, p[0], p[1], p[2]))

        else:
            self.get_logger().error("No PointCloud data available for processing.")

        #Publish centroid pose
        #self.centroid_publisher_.publish(self.centroid_)

        return response


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
