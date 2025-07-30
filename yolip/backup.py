from scripts.utils import logger, find_nearest
import scripts.yolo as yolo
import rclpy
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from messages.msg import Vision

class yolo_node:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('yolo_node')
        self.sub = self.node.create_subscription(Image, '/camera/ground', self.ground_cb, 10)
        self.pub = self.node.create_publisher(Vision, 'vision', 10)
        self.bridge = CvBridge()
        # 模型预热
        input = np.zeros((640, 640, 3), dtype=np.uint8)
        yolo.infer_cut(input)
        logger.info("YOLO节点初始化完成")

    def default_vision(self):
        return Vision(
            is_detected = False,
            center_x = 0,
            center_y = 0,
            label = "None",
        )

    def ground_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.node.get_logger().error(f"ROS图像转换失败: {e}")
            self.pub.publish(self.default_vision())
            return

        vision_msg = self.default_vision()
        result = self.process(cv_image, vision_msg)
        self.pub.publish(vision_msg)

    def process(self, img, msg):
        cut_results = yolo.infer_cut(img)
        if not cut_results:
            logger.info("YOLO未检测到目标")
            return False

        # 选出最近的目标框坐标，填充ros消息
        cut_result = find_nearest(cut_results)
        msg.is_detected = True
        msg.center_x, msg.center_y = cut_result.get_center_point()
        # label保持默认值"None"
        
        logger.info(f"YOLO检测到目标，中心点: ({msg.center_x}, {msg.center_y})")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = yolo_node()
    try:
        rclpy.spin(node.node)
    except KeyboardInterrupt:
        logger.info("节点被用户中断")
    finally:
        node.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()