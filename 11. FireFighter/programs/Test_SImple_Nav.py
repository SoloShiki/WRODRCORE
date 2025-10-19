#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class SimpleGoal(Node):
    def __init__(self):
        super().__init__('simple_goal_sender')
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(2.0, self.send_goal)

    def send_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'base_link'  # 2 m ahead of robot
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = 2.0
        goal.pose.orientation.w = 1.0
        self.pub.publish(goal)
        self.get_logger().info('Sent goal 2 m forward')
        self.timer.cancel()

def main():
    rclpy.init()
    node = SimpleGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
