#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import math

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for Nav2 to become active
    navigator.waitUntilNav2Active()

    # Create goal
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = 2.0
    goal.pose.orientation.w = 1.0

    # Send goal
    navigator.goToPose(goal)
    print("Sent navigation goal 2 meters ahead in map frame")

    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.1)

    result = navigator.getResult()
    print(f"Navigation result: {result}")

    if result == navigator.ResultStatus.SUCCEEDED:
        print("✅ Goal reached successfully!")
    elif result == navigator.ResultStatus.CANCELED:
        print("⚠️ Goal was canceled.")
    elif result == navigator.ResultStatus.FAILED:
        print("❌ Goal failed.")
    else:
        print("Unknown result.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
