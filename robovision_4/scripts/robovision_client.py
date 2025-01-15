#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from robovision_interfaces.msg import ObjectCentroid
from robovision_interfaces.srv import GetPointCenter

import numpy as np
import traceback


class PointCloudCentroidNode(Node):
    def __init__(self):
        super().__init__("point_cloud_client")

        #Declare parameters
        self.declare_parameter("x", 320)
        self.declare_parameter("y", 240)

        #Get parameter values
        self.x_ = self.get_parameter("x").value
        self.y_ = self.get_parameter("y").value

        self.point_ = []

        #Processing timer
        #We call our service once each 2.5 seconds
        self.client_call_timer_ = self.create_timer(2.5, self.client_caller)

        self.get_logger().info("Starting point_cloud_client application in python...")

    def client_caller(self):
        
        self.call_get_point_center_server(self.x_, self.y_)

        #Do something with the result
        if (self.point_):
            self.get_logger().info("(" + str(self.x_) + ", " + str(self.y_) + ") position is: " + str(self.point_))
            self.point_ = []

    #Basic server call template
    def call_get_point_center_server(self, _x, _y):
        client = self.create_client(GetPointCenter, "get_point_center")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for get_point_center service...")

        request = GetPointCenter.Request()
        request.x = _x
        request.y = _y

        future = client.call_async(request)

        #The callback only contains the response;
        #we explicity add the request for debugging purposes
        future.add_done_callback(
            partial(self.callback_call_point_cloud, _x=request.x, _y=request.y))
    
    def callback_call_point_cloud(self, future, _x, _y):
        try:
            response = future.result()
            self.get_logger().info("(" + str(_x) + ", " + str(_y) + ") position is: " + str(response.point))

            self.point_ = response.point
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudCentroidNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
