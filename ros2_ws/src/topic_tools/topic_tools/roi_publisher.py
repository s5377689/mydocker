import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ROIPublisher(Node):
    def __init__(self):
        super().__init__('roi_publisher')

        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.publisher_ = self.create_publisher(Image, '/still_img', qos_profile)

        self.bridge = CvBridge()

        # cap = cv2.VideoCapture("rtsp://admin:53373957@192.168.144.108:554/cam/realmonitor?channel=1&subtype=0")
        # ret, frame = cap.read()
        # if not ret:
        #     self.get_logger().error("Could not read from camera.")
        #     return

        # frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        # roi = cv2.selectROI("Select ROI", frame, showCrosshair=False)
        # cv2.destroyAllWindows()
        # x, y, w, h = map(int, roi)

        # TEMP for testing
        # x, y, w, h = 50, 50, 250, 250
        frame = cv2.imread("/home/robuff/data/shiba/shiba-full.jpg") 
        frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        roi = cv2.selectROI("Select ROI", frame, showCrosshair=False)
        cv2.destroyAllWindows()
        x, y, w, h = map(int, roi)

        if w == 0 or h == 0:
            self.get_logger().warn("No ROI selected.")
            return

        cropped = frame[y:y+h, x:x+w]
        msg = self.bridge.cv2_to_imgmsg(cropped, encoding="bgr8")
        self.publisher_.publish(msg)
        self.get_logger().info("Published selected ROI.")

        # cap.release()

def main(args=None):
    rclpy.init(args=args)
    roi_publisher = ROIPublisher()
    rclpy.spin(roi_publisher)
    roi_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()