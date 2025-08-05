import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import cv2
import cv_bridge
import numpy as np
import sys

class ImageDecoder(Node):
    def __init__(self):
        super().__init__('img_decoder')

        # self.declare_parameter('topic_name', '/camera/compressed/image_raw')
        self.declare_parameter('topic_name', '/camera/compressed/image_tracking')
        self.declare_parameter('republish', True)
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.republish = self.get_parameter('republish').get_parameter_value().bool_value

        self.subscription = self.create_subscription(
            CompressedImage,
            self.topic_name,
            self.img_callback,
            10
        )
        self.get_logger().info("Subscribed to {}".format(self.topic_name))

        if self.republish:
            self.bridge = cv_bridge.CvBridge()
            new_topic_name = self.topic_name.replace('/compressed', '')
            self.publisher = self.create_publisher(
                Image,
                new_topic_name,
                10)
            self.get_logger().info("Republishing to {}".format(new_topic_name))

    def img_callback(self, msg):
        try:
            # Convert byte data to numpy array
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)

            # Decode image
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if image is None:
                self.get_logger().warn("Decoded image is None.")
                return

            # Show the image
            cv2.imshow("Compressed Image Viewer", image)
            cv2.waitKey(1)

            # Republish the image if required
            if self.republish:
                # Publish the image
                img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                img_msg.header = msg.header
                self.publisher.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to decode image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()