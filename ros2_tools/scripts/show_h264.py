# 未知内容 25.1.21

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np

class H264ReceiverNode(Node):
    def __init__(self):
        super().__init__('h264_receiver_node')

        self.subscription = self.create_subscription(
            String,
            '/camera/ground_h264',
            self.callback,
            10
        )
        self.subscription

        self.decoder = cv2.VideoCapture('appsrc ! h264parse ! avdec_h264 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
        if not self.decoder.isOpened():
            self.get_logger().error('Failed to initialize H.264 decoder.')

    def callback(self, msg):
        try:
            h264_data = bytes.fromhex(msg.data)
            self.decoder.write(h264_data)
            success, frame = self.decoder.read()
            if success:
                cv2.imshow("H264 Stream", frame)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')

if __name__ == '__main__':
    rclpy.init(args=None)
    node = H264ReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
