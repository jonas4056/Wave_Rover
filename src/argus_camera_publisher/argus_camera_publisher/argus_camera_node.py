#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ArgusCameraNode(Node):
    def __init__(self):
        super().__init__('argus_camera_node')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.br = CvBridge()

        # Adjust resolution and framerate as needed
        self.cap = cv2.VideoCapture(
            "nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 "
            "! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink",
            cv2.CAP_GSTREAMER
        )

        if not self.cap.isOpened():
            self.get_logger().error("Could not open nvarguscamerasrc pipeline")
            return

        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30Hz
        self.get_logger().info("Argus camera node started!")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Frame capture failed")
            return

        msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArgusCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
