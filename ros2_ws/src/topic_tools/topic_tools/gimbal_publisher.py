import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import subprocess
import numpy as np
import cv2
import threading


class GimbalPublisher(Node):
    def __init__(self):
        super().__init__('gimbal_publisher_node')

        rtsp_eo_url = "rtsp://admin:53373957@192.168.144.108:554/cam/realmonitor?channel=1&subtype=0"
        rtsp_ir_url = "rtsp://192.168.144.6:8554"

        self.declare_parameter('camera', 'eo')  # 'eo' or 'ir'

        camera_type = self.get_parameter('camera').get_parameter_value().string_value

        if camera_type == 'eo':
            self.url = rtsp_eo_url
            self.width = 1920
            self.height = 1080
            self.topic_name = '/camera/image_raw'
        elif camera_type == 'ir':
            self.url = rtsp_ir_url
            self.width = 640
            self.height = 480
            self.topic_name = '/camera/ir_image_raw'
        else:
            self.get_logger().error(f'Invalid camera type: {camera_type}. Use "eo" or "ir".')
            rclpy.shutdown()
            return

        self.publisher = self.create_publisher(Image, self.topic_name, 10)
        self.bridge = CvBridge()

        self.get_logger().info(f'Starting gimbal stream from {self.url}')
        self.running = True

        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()

    def capture_loop(self):
        cmd = [
            'ffmpeg',
            '-i', self.url,
            '-loglevel', 'quiet',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-'
        ]

        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**8)
        frame_size = self.width * self.height * 3

        try:
            while self.running:
                raw_frame = process.stdout.read(frame_size)
                if not raw_frame:
                    self.get_logger().warn('Empty frame received.')
                    break

                frame = np.frombuffer(raw_frame, np.uint8).reshape((self.height, self.width, 3))
                frame = cv2.resize(frame, None, fx=0.25, fy=0.25)
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error reading ffmpeg stream: {e}')
        finally:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
            if process.stdout:
                process.stdout.close()

    def destroy_node(self):
        self.running = False
        self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GimbalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()