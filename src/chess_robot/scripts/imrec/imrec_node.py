#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import time
from std_srvs.srv import Trigger
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from imrec import imrec

BASE_PATH = "/home/mates/onlab/src/chess_robot/scripts/imrec"
MODEL_PATH = os.path.join(BASE_PATH, "chess_brain.pth")


class ChessVisionNode(Node):
    def __init__(self):
        super().__init__('chess_vision_node')
        self.get_logger().info("Node has started!")
        
        self.cb_group = ReentrantCallbackGroup()

        self.get_logger().info("Loading...")
        self.vision = imrec(MODEL_PATH)
        self.get_logger().info("Model loaded")

        self.fen_service = self.create_service(Trigger, 'get_chess_fen', self.get_fen_callback, callback_group=self.cb_group)
        self.tile_service = self.create_service(Trigger, 'get_tile', self.get_tile_callback, callback_group=self.cb_group)

        self.mc_trigger = self.create_client(Trigger, 'get_latest_frame', callback_group=self.cb_group)

        self.bridge = CvBridge()
        self.latest_frame = None

        self.image_subscribtion = self.create_subscription(
            Image,
            '/camera_latest',
            self.image_callback,
            10,
            callback_group=self.cb_group
        )
        self.get_logger().info("Subscribed to /camera_latest.")

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to translate ROS image message: {str(e)}")

    def fetch_new_image_from_mc(self):
        self.latest_frame = None
        req = Trigger.Request()
        self.mc_trigger.call(req)
        
        timeout = 2.0
        start_time = time.time()
        while self.latest_frame is None and (time.time() - start_time) < timeout:
            time.sleep(0.05)
            
        return self.latest_frame is not None

    def get_fen_callback(self, request, response):
        self.get_logger().info("Requesting fresh image from MC...")
        
        if not self.fetch_new_image_from_mc():
            self.get_logger().error("MC did not send an image!")
            response.success = False
            return response

        self.get_logger().info("Image received. Processing FEN...")
        fen = self.vision.get_board(self.latest_frame)
        
        if fen:
            response.success = True
            response.message = fen
            self.get_logger().info(f"Detected FEN: {fen}")
        else:
            response.success = False
        
        return response
    
    def get_tile_callback(self, request, response):
        self.get_logger().info("Requesting fresh image from MC for Tile Centers...")
        
        if not self.fetch_new_image_from_mc():
            self.get_logger().error("MC did not send an image!")
            response.success = False
            response.message = "Failed to get image from MC"
            return response
            
        self.get_logger().info("Image received. Processing Tile Centers...")
        payload, scale_h, scale_v = self.vision.get_centers(self.latest_frame)
        
        if payload:
            response.success = True
            height, width = self.latest_frame.shape[:2]
            mid_x = width / 2.0
            mid_y = height / 2.0

            # Append the two scale values to the end of the coordinate string
            response.message = f"{payload} {scale_h:.2f} {scale_v:.2f} {mid_x:.1f} {mid_y:.1f}"
            self.get_logger().info("Tile centers and scales retrieved.")
        else:
            response.success = False
            response.message = "Failed to detect markers for centers"
            
        return response


def main():
    rclpy.init()
    node = ChessVisionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()