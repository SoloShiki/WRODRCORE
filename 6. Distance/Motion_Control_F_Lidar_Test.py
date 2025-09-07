import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_tester')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',   # LiDAR topic (change if your robot uses another)
            self.lidar_callback,
            10
        )
        self.get_logger().info("✅ Lidar Tester Node started. Listening to /scan...")

    def lidar_callback(self, msg: LaserScan):
        ranges = msg.ranges

        # Convert angles to indexes
        def get_range(angle_deg, window=5):
            """Get the min distance around a specific angle (±window degrees)"""
            angle_rad = math.radians(angle_deg)
            index = int((angle_rad - msg.angle_min) / msg.angle_increment)
            half_window = int(window / msg.angle_increment)

            # Clamp indexes
            start = max(0, index - half_window)
            end = min(len(ranges) - 1, index + half_window)

            # Filter valid readings
            values = [r for r in ranges[start:end] if not math.isinf(r) and not math.isnan(r)]
            return min(values) if values else float('inf')

        # Read different directions
        front = get_range(0)       # 0° = straight ahead
        left = get_range(90)       # 90° = left
        right = get_range(-90)     # -90° = right
        back = get_range(180)      # 180° = behind

        self.get_logger().info(
            f"📡 Distances | Front: {front:.2f} m | Left: {left:.2f} m | Right: {right:.2f} m | Back: {back:.2f} m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
