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

    # 🚀 Movement helper functions
    def forward(self, speed=0.2, duration=2.0):
        """Move forward with given speed and duration"""
        self.send_twist(linear_x=speed, angular_z=0.0, duration=duration)

    def backward(self, speed=0.2, duration=2.0):
        """Move backward with given speed and duration"""
        self.send_twist(linear_x=-speed, angular_z=0.0, duration=duration)

    def turn_left(self, speed=0.5, duration=2.0, angle=5.0):
        """Rotate left (counter-clockwise)"""
        self.send_twist(linear_x=speed, angular_z=angle, duration=duration)

    def turn_right(self, speed=0.5, duration=2.0, angle=5.0):
        """Rotate right (clockwise)"""
        self.send_twist(linear_x=speed, angular_z=-angle, duration=duration)
        
       # 🛑 Stop function
    def stop(self, duration=1.0):
        """Stop all movement, including steering"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.1)



def main():
    rclpy.init()
    node = CmdVelPublisher()

    # Example usage:
    node.forward(0.3, 5)    # Move forward
    node.turn_left(0.5, 3)  # Turn left
    node.backward(0.3, 2)   # Move backward
    node.turn_right(0.5, 5) # Turn right

   # Stop after actions
    node.stop(1)  # Stop all movement for 1 second

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
