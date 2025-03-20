# 未知内容 25.1.21

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from threading import Thread
import cv2
import numpy as np
import subprocess
import signal
import atexit

class ImageH264Streamer(Node):
    def __init__(self):
        super().__init__('image_h264_streamer')
        self.subscribers = {}
        self.image_publishers = {}
        self.processes = {}
        self.timer = self.create_timer(1.0, self.update_topics)
        atexit.register(self.cleanup)

    def update_topics(self):
        topics = self.get_topic_names_and_types()
        image_topics = [name for name, types in topics if 'sensor_msgs/msg/Image' in types]

        for topic in image_topics:
            if topic not in self.subscribers:
                self.get_logger().info(f'Subscribing to new topic: {topic}')
                subscriber = self.create_subscription(Image, topic, lambda msg, t=topic: self.image_callback(msg, t), 10)
                self.subscribers[topic] = subscriber
                if not topic.endswith('_h264'):
                    h264_topic = f'{topic}_h264'
                else:
                    h264_topic = topic
                publisher = self.create_publisher(Image, h264_topic, 10)
                self.image_publishers[h264_topic] = publisher
                self.processes[topic] = self.start_gstreamer_process()

        for topic in list(self.subscribers.keys()):
            if topic not in image_topics:
                self.get_logger().info(f'Unsubscribing from topic: {topic}')
                self.destroy_subscription(self.subscribers.pop(topic))
                self.image_publishers.pop(topic)
                self.stop_gstreamer_process(self.processes.pop(topic))

    def image_callback(self, msg, topic):
        img = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        ret, jpeg = cv2.imencode('.jpg', img)
        if not ret:
            self.get_logger().error(f'Failed to encode image from topic: {topic}')
            return
        process = self.processes.get(topic)
        if process:
            try:
                process.stdin.write(jpeg.tobytes())
                process.stdin.flush()
            except Exception as e:
                self.get_logger().error(f'Failed to write to GStreamer process for topic {topic}: {e}')

        self.get_logger().info('123')
        if topic in self.image_publishers:
            self.image_publishers[topic].publish(msg)
        process = self.processes.get(topic)
        if process:
            try:
                process.stdin.write(jpeg.tobytes())
                process.stdin.flush()
            except Exception as e:
                self.get_logger().error(f'Failed to write to GStreamer process for topic {topic}: {e}')

    def start_gstreamer_process(self):
        gst_command = [
            'gst-launch-1.0', 'fdsrc', '!', 'jpegdec', '!', 'x264enc', 'tune=zerolatency', '!', 'rtph264pay', '!', 'udpsink', 'host=127.0.0.1', 'port=5000'
        ]
        process = subprocess.Popen(gst_command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        return process

    def stop_gstreamer_process(self, process):
        if process:
            process.send_signal(signal.SIGINT)
            process.wait()

    def cleanup(self):
        for process in self.processes.values():
            self.stop_gstreamer_process(process)


if __name__ == '__main__':
    rclpy.init(args=None)
    node = ImageH264Streamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
