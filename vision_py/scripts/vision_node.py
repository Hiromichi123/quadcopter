#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node, Rate
from vision.msg import Vision
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv_tools # 导入工具类

class vision_pub_node(Node):
    def __init__(self):
        super().__init__('vision_pub')
        self.vision_pub = self.create_publisher(Vision, 'vision', 10)
        #self.frame_pub1 = self.create_publisher(CompressedImage, 'frame1', 10)
        #self.frame_pub2 = self.create_publisher(CompressedImage, 'frame2', 10)
        self.frame_sub = self.create_subscription(Image, '/camera/ground', self.ground_callback, 1) # 实机
        #self.frame_sub = self.create_subscription(Image, '/camera_ground/image_raw', self.ground_callback, 1) # 仿真
        self.bridge = CvBridge()
        self.cv_tools = cv_tools.CVTools(self)  # 创建工具类实例
        # 初始化消息
        self.msg = Vision()
        self.msg.is_line_detected = False
        self.msg.lateral_error = 0
        self.msg.angle_error = 0.0
        self.msg.is_square_detected = False
        self.msg.center_x1_error = 0
        self.msg.center_y1_error = 0
        self.msg.is_circle_detected = False
        self.msg.center_x2_error = 0
        self.msg.center_y2_error = 0
        self.get_logger().info("init complete")
    
    def ground_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error convert image: {e}")
            return
        self.process(cv_image) # 处理并发布vision消息

    def process(self, frame):
        try:            
            #bl_frame = self.cv_tools.backlight_compensation(frame) # 逆光补偿
            #cv2.imshow('逆光补偿效果', bl_frame)

            #_, thresh_frame2 = cv2.threshold(gray_frame, 235, 255, cv2.THRESH_BINARY)
            #contours, _ = cv2.findContours(thresh_frame2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  # 提取轮廓
            #valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1000]
            #cv2.drawContours(frame, valid_contours, -1, (0, 255, 0), 3)  # 绘制轮廓
            #detect_copy = self.cv_tools.detect_contours(valid_contours, bl_frame)  # 过滤轮廓，并检测
            #cv2.imshow('图形检测效果', detect_copy)
            #cv2.imshow('vision', detect_copy)

            detect_copy2 = self.cv_tools.red_circle_detect(frame)  # 红色圆形检测
            #cv2.imshow('red', detect_copy2)
            detect_copy1 = self.cv_tools.yellow_square_detect(frame)  # 矩形检测
            #cv2.imshow('yellow', detect_copy1)
            #cv2.waitKey(1)

            #self.frame_pub1.publish(self.bridge.cv2_to_compressed_imgmsg(detect_copy1))
            #self.frame_pub2.publish(self.bridge.cv2_to_compressed_imgmsg(hl_copy))

            frame = frame[:, frame.shape[1] // 6 : -frame.shape[1] // 6]
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # 灰度
            _, thresh_frame1 = cv2.threshold(gray_frame, 100, 255, cv2.THRESH_BINARY) # 二值化处理，<50->0，>50->255
            hl_copy = self.cv_tools.line_detect(thresh_frame1) # 霍夫直线
            #cv2.imshow('霍夫直线效果', hl_copy)

            self.vision_pub.publish(self.msg)

        except Exception as e:
            self.get_logger().error(f"Error occurred: {e}")

    def release_resources(self):
        self.get_logger().info("Resources released.")

# entry_point入口
def main():
    rclpy.init(args=None)
    node = vision_pub_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shutting down.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()