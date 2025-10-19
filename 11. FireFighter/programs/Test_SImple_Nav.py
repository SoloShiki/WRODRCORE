#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
import math

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for Nav2 to fully activate
    navigator.waitUntilNav2Active()

    # Create a goal pose in the map frame
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = 2.0
    goal.pose.position.y = 0.0
    goal.pose.orientation.w = 1.0

    navigator.goToPose(goal)

    # Wait for result
    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.1)

    result = navigator.getResult()
    if result == NavigationResult.SUCCEEDED:
        print('Reached goal successfully!')
    else:
        print('Navigation failed or canceled.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
