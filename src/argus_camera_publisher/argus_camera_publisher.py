#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ArgusCameraPublisher(Node):
    def __init__(self):
        super().__init__('argus_camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open nvarguscamerasrc pipeline')
        else:
            self.get_logger().info('Camera opened successfully')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('No frame captured')

def main(args=None):
    rclpy.init(args=args)
    node = ArgusCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
