import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleMove(Node):
    def __init__(self):
        super().__init__('simple_move')
        # Use the MentorPi controller topic
        self.publisher = self.create_publisher(Twist, '/controller/cmd_vel', 10)

        # Timer tick every 0.1s (10 Hz)
        self.timer = self.create_timer(0.1, self.update)

        # State machine
        self.state = "moving"
        self.state_start = self.get_clock().now()
        self.get_logger().info('Robot starting movement for 3 seconds...')

    def update(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start).nanoseconds / 1e9  # seconds
        msg = Twist()

        if self.state == "moving":
            msg.linear.x = 0.2  # forward speed
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            if elapsed >= 3.0:
                self.next_state("stopped")

        elif self.state == "stopped":
            # Explicitly stop the robot
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            # Cancel the timer to stop publishing
            self.timer.cancel()
            self.get_logger().info("Robot stopped. Sequence complete.")

    def next_state(self, new_state):
        self.state = new_state
        self.state_start = self.get_clock().now()
        self.get_logger().info(f"Switching to state: {new_state}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMove()
    rclpy.spin(node)
    # Before shutdown, ensure robot is stopped
    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    stop_msg.angular.z = 0.0
    node.publisher.publish(stop_msg)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
