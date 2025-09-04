import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class CmdVelDriver(Node):
    def __init__(self):
        super().__init__('cmd_vel_driver')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(5.0, self.drive_sequence)  # every 5 seconds
        self.get_logger().info('CmdVelDriver node started.')

    def drive_sequence(self):
        # Step 1: Move forward 2 meters at 0.2 m/s
        forward_msg = Twist()
        forward_msg.linear.x = 0.2
        self.publisher.publish(forward_msg)
        self.get_logger().info('Moving forward...')
        time.sleep(10)  # 2m / 0.2 m/s = 10s

        # Step 2: Stop
        stop_msg = Twist()
        self.publisher.publish(stop_msg)
        time.sleep(1)

        # Step 3: Turn 10° left at 0.1 rad/s
        turn_msg = Twist()
        turn_msg.angular.z = 0.1
        self.publisher.publish(turn_msg)
        self.get_logger().info('Turning left...')
        time.sleep(math.radians(10) / 0.1)  # ≈1.75s

        # Step 4: Stop again
        self.publisher.publish(stop_msg)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
