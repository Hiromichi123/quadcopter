# yolo框选 + clip识别版
import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
import queue

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from messages.msg import Vision2

from scripts.utils import find_nearest
import scripts.yolo as yolo
import scripts.clip as clip

class yolip_node(Node):
    def __init__(self):
        self.image_queue = queue.Queue(maxsize=2)  # 保存2帧
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

        # 线程变量
        self.latest_rgb_img = None
        self.img_lock = threading.Lock()
        self.infer_thread = threading.Thread(target=self.infer_worker, daemon=True)
        self.infer_thread.start()

        self.pub = self.create_publisher(Vision2, 'vision2', 10)
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

        # 模型预热
        input = np.zeros((640, 640, 3), dtype=np.uint8) # 空图像
        yolo.infer_cut(input)
        clip.infer(input)
        self.get_logger().info("yolip_node初始化")

    def default_vision(self):
        return Vision2(
            is_detected=False, # bool
            type=0, # int32 1-5代表种类
            number=0, # int32 检测到的目标数量
            region=0, # uint8 当前区域
        )

    # rgb回调
    def rgb_cb(self, msg):
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            try:
                self.image_queue.put_nowait(rgb_img.copy()) # 非阻塞放入队列
            except queue.Full:
                try:
                    self.image_queue.get_nowait()
                    self.image_queue.put_nowait(rgb_img.copy()) # 满了移除最老帧
                except queue.Empty:
                    pass
        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败: {e}")

    def infer_worker(self):
        while True:
            try:
                # 非阻塞获取图像
                try:
                    rgb_img = self.image_queue.get_nowait()
                except queue.Empty:
                    time.sleep(0.01)
                    continue
                
                # 处理图像（无锁）
                img_copy = rgb_img.copy()
                vision_msg = self.default_vision()
                result = self.classify(rgb_img, img_copy, vision_msg)

                cv2.imshow("yolo", img_copy)
                cv2.waitKey(1)
                
                self.pub.publish(vision_msg)
                    
            except Exception as e:
                self.get_logger().error(f"推理线程异常: {e}")
                time.sleep(1.0)

    # 进行双重识别
    def classify(self, img, img_copy, msg):
        cut_results, msg.number = yolo.infer_cut(img)
        if not cut_results:
            return False

        msg.is_detected = True
        cut_result = find_nearest(cut_results, img.shape) # 选出最近的动物yolo框
        yolo.draw_yolo_box(img_copy, cut_result) # 绘制yolo矩形框

        #进行CLIP识别
        clip_results = clip.infer(cut_result.square_cut_img)
        probs = {}
        for clip_result in clip_results:
            if clip_result.confidence > 0.2:  # 置信度阈值
                probs[clip_result.name] = clip_result.confidence
        
        if not probs:
            self.get_logger().info("CLIP未识别出有效目标")
            return False

        result = max(probs, key=probs.get)
        self.get_logger().info(f"CLIP识别结果: {result} (置信度: {probs[result]:.2f})")
        msg.type = self.OBJECT_TYPE_MAPPING.get(result, 0)
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
