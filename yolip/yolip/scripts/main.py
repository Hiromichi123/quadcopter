# yolo框选 + clip识别版
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
from collections import deque

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, UInt8MultiArray, Empty
from nav_msgs.msg import Odometry
from messages.msg import Vision

from . import yolo
from . import clip

pic_num = 0

class yolip_node(Node):
    def __init__(self):
        # 动物字典映射
        self.OBJECT_TYPE_MAPPING = {
            '大象': 1,
            '老虎': 2,
            '狼': 3,
            '猴子': 4,
            '孔雀': 5
        }

        super().__init__('yolip_node')
        self.bridge = CvBridge()

        # 图像缓存最新3帧
        self.image_cache = deque(maxlen=3)
        self.odom_msg = None

        self.pub = self.create_publisher(UInt8MultiArray, 'region', 10)
        self.ready_pub = self.create_publisher(Empty, 'ready', 10)
        self.odom_pub = self.create_publisher(Vision, 'vision', 10)
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_cb,
            qos_profile
        )
        self.region_sub = self.create_subscription(
            UInt8,
            '/detect',
            self.region_cb,
            1
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_cb,
            1
        )

        # 预热
        self.get_logger().info("开始模型预热...")
        input = np.zeros((960, 960, 3), dtype=np.uint8) # 空图像
        yolo.infer_cut(input)
        clip.infer(input)
        self.get_logger().info("yolip_node初始化完成")
        self.ready_pub.publish(Empty())  # 发布ready信号
        rclpy.spin_once(self, timeout_sec=0.1)

    # rgb回调
    def rgb_cb(self, msg):
        try:
            bgr_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 添加到缓存（自动保持最新3帧）
            self.image_cache.append(bgr_img.copy())
        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败: {e}")

    # odom回调
    def odom_cb(self, msg):
        self.odom_msg = msg

    # 区域触发回调
    def region_cb(self, msg):
        global pic_num
        if msg.data == 0:
            return

        region_num = msg.data
        self.get_logger().info(f"当前区域: {region_num}，开始识别处理")

        # 检查缓存中是否有图像
        if not self.image_cache:
            self.get_logger().warn("触发识别但图像缓存为空")
            msg_array = UInt8MultiArray()
            msg_array.data = [region_num, 255] # 空触发记号
            self.pub.publish(msg_array)
            return
        
        # 获取最新的图像
        latest_frame = self.image_cache[-1]
        rgb_img = latest_frame.copy()
        try:
            img_copy = rgb_img.copy()
            result = self.classify(rgb_img, img_copy, region_num)
            
            # 显示结果
            cv2.imshow("yolo_result", img_copy)
            cv2.imwrite(f"./yolo_{pic_num}.jpg", img_copy)
            cv2.waitKey(1)
            pic_num += 1

        except Exception as e:
            # 发布失败结果
            self.get_logger().error(f"识别处理异常: {e}")
            msg_array = UInt8MultiArray()
            msg_array.data = [region_num, 250]  # 失败记号
            self.pub.publish(msg_array)

    # 进行双重识别
    def classify(self, img, img_copy, region_num):
        # 绘制画面中心点
        cv2.circle(img_copy, (640, 360), 5, (0, 255, 0), -1)

        # 及时获取odom信息
        odom_x = self.odom_msg.pose.pose.position.x
        odom_y = self.odom_msg.pose.pose.position.y

        # 进行yolo剪裁
        cut_results = yolo.infer_cut(img)
        if not cut_results:
            return False

        # 绘制yolo矩形框
        yolo.draw_all_yolo_boxes(img_copy, cut_results)

        #对每个yolo框进行CLIP识别
        for cut_result in cut_results:
            if not cut_result:
                continue

            clip_results = clip.infer(cut_result.square_cut_img)
            probs = {}
            for clip_result in clip_results:
                if clip_result.confidence > 0.3:  # 置信度阈值
                    probs[clip_result.name] = clip_result.confidence

            # 低置信度，跳过该yolo框
            if not probs:
                continue

            # 成功识别
            result = max(probs, key=probs.get)
            confidence = probs[result]
            self.get_logger().info(f"CLIP识别结果: {result}, 置信度: {confidence:.2f}")
            clip.draw_clip_result(img_copy, cut_result, result, confidence) # 绘制CLIP识别结果

            (center_x, center_y) = cut_result.get_center_point()
            # 正确发布识别结果11-96区域代码
            msg_array = UInt8MultiArray()
            msg_array.data = [region_num, self.OBJECT_TYPE_MAPPING.get(result, 0)]
            self.pub.publish(msg_array)
            Vision_msg = Vision()
            Vision_msg.center_x = center_x
            Vision_msg.center_y = center_y
            Vision_msg.odom_x = odom_x
            Vision_msg.odom_y = odom_y
            self.odom_pub.publish(Vision_msg)

        return True

def main(args=None):
    rclpy.init(args=args)
    node = yolip_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("yolip_node被中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
