import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
import threading
import sys
import select
import tty
import termios


class KeyboardPublisherNode(Node):
    def __init__(self):
        super().__init__('keyboard_publisher_node')
        self.publisher = self.create_publisher(UInt8, 'detect', 10)
        self.get_logger().info('Keyboard Publisher Node has been started.')
        self.get_logger().info('Press SPACE to publish message, ESC or Ctrl+C to quit.')
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()
    
    def publish_message(self):
        """发布消息"""
        msg = UInt8()
        msg.data = 20
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data} to topic "detect"')
    
    def keyboard_listener(self):
        """键盘监听线程"""
        # 保存原始终端设置
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # 设置终端为原始模式
            tty.setraw(sys.stdin.fileno())
            
            while rclpy.ok():
                # 检查是否有键盘输入
                if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                    char = sys.stdin.read(1)
                    
                    if char == ' ':  # 空格键
                        self.publish_message()
                    elif char == '\x1b':  # ESC键
                        self.get_logger().info('ESC pressed, shutting down...')
                        rclpy.shutdown()
                        break
                    elif char == '\x03':  # Ctrl+C
                        self.get_logger().info('Ctrl+C pressed, shutting down...')
                        rclpy.shutdown()
                        break
                        
        except Exception as e:
            self.get_logger().error(f'Keyboard listener error: {e}')
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()