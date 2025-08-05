import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from rosgraph_msgs.msg import Clock

class CmdVel2Ap(Node):
    def __init__(self):
        super().__init__('cmd_vel2ap')
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.create_subscription(
            Clock,
            '/ap/clock',
            self.clock_callback,
            10
        )
        self.publisher = self.create_publisher(
            TwistStamped, 
            '/ap/cmd_vel',
            10
        )
        self.current_clock =  Clock()

    def cmd_vel_callback(self, msg):
        new_msg = TwistStamped()
        new_msg.header.stamp = self.current_clock.clock
        new_msg.header.frame_id = "base_link"
        new_msg.twist = msg
        self.publisher.publish(new_msg)
    
    def clock_callback(self, msg):
        self.current_clock = msg 

def main(args=None):
    rclpy.init(args=args)
    cmd_vel2ap = CmdVel2Ap()
    rclpy.spin(cmd_vel2ap)
    cmd_vel2ap.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
