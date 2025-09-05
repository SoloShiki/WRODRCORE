import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)

    def send_twist(self, linear_x, angular_z, duration):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.1)

def main():
    rclpy.init()
    node = CmdVelPublisher()

    # Move forward for 2 seconds
    node.send_twist(0.2, 0.0, 2)

    # Stop for 2 seconds
    node.send_twist(0.0, 0.0, 2)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()