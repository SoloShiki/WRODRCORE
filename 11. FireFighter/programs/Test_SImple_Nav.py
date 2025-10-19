#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
import math

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for Nav2 to become active
    navigator.waitUntilNav2Active()

    # Create a goal pose in the MAP frame
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()

    # Example: move 2 meters forward along +X direction in map frame
    goal.pose.position.x = 2.0
    goal.pose.position.y = 0.0
    goal.pose.orientation.w = 1.0

    # Send goal to Nav2
    navigator.goToPose(goal)
    print("Sent navigation goal 2 meters ahead in map frame")

    # Wait for completion
    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.1)

    # Check result
    result = navigator.getResult()
    if result == NavigationResult.SUCCEEDED:
        print("✅ Goal reached successfully!")
    elif result == NavigationResult.CANCELED:
        print("⚠️ Goal was canceled.")
    elif result == NavigationResult.FAILED:
        print("❌ Goal failed.")
    else:
        print("Unknown result.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
