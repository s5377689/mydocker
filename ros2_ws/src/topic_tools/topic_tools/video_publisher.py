import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        self.video_path = self.declare_parameter('video_path', '/home/robuff/data/ship-videos/test.avi').value
        self.loop = self.declare_parameter('loop', True).value
        self.video_pub = self.create_publisher(
            Image,
            self.image_topic,
            10
        )
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.video_path)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video file: {self.video_path}")
            rclpy.shutdown()
            return

        timer_period = 1 / self.cap.get(cv2.CAP_PROP_FPS)
        self.timer = self.create_timer(
            timer_period,
            self.timer_cb
        )

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            if self.loop:
                self.get_logger().info("Looping video.")
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
            else: 
                self.get_logger().info("End of video file reached.")
                self.cap.release()
                rclpy.shutdown()
                return

        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.video_pub.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()