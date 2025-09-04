import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class CmdVelDriver(Node):
    def __init__(self):
        super().__init__('cmd_vel_driver')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer: tick every 0.1s
        self.timer = self.create_timer(0.1, self.update)

        # State machine
        self.state = "forward"
        self.state_start = self.get_clock().now()

        self.get_logger().info('CmdVelDriver node started.')

    def update(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start).nanoseconds / 1e9  # seconds

        msg = Twist()

        if self.state == "forward":
            msg.linear.x = 0.2
            self.publisher.publish(msg)
            if elapsed >= 10.0:  # 2m / 0.2 m/s = 10s
                self.next_state("stop1")

        elif self.state == "stop1":
            # stop
            self.publisher.publish(msg)  # all zeros
            if elapsed >= 2.0:
                self.next_state("turn")

        elif self.state == "turn":
            msg.angular.z = 0.1
            self.publisher.publish(msg)
            if elapsed >= math.radians(10) / 0.1:  # ~1.75s
                self.next_state("stop2")

        elif self.state == "stop2":
            self.publisher.publish(msg)  # stop again
            if elapsed >= 2.0:
                self.get_logger().info("Sequence complete.")
                self.destroy_node()

    def next_state(self, new_state):
        self.state = new_state
        self.state_start = self.get_clock().now()
        self.get_logger().info(f"Switching to state: {new_state}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
