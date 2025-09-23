#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading


class FireAlertSubscriber(Node):
    def __init__(self):
        super().__init__('fire_alert_subscriber')

        # Subscribe to fire alerts
        self.subscription = self.create_subscription(
            String,
            'fire_alert',
            self.listener_callback,
            10
        )

        self.get_logger().info('🔥 Subscriber listo para recibir alertas de fuego')

    def listener_callback(self, msg):
        self.get_logger().info(f'⚠️ Alerta recibida: {msg.data}')

        # Only act if fire is detected
        if "Fuego" in msg.data or "fire" in msg.data.lower():
            self.get_logger().info("🚗 Ejecutando driving.py por alerta de fuego")

            # Run in separate thread so callback is not blocked
            thread = threading.Thread(target=self.run_driving, daemon=True)
            thread.start()

    def run_driving(self):
        try:
            result = subprocess.run(
            #FIX THE PATH
                ["python3", "/home/ubuntu/motion/src/motion-test/scripts/WROS_RD_ROS/driving.py"],
                capture_output=True,
                text=True
            )
            self.get_logger().info(f"✅ driving.py terminó con salida:\n{result.stdout}")

            if result.stderr:
                self.get_logger().warn(f"⚠️ driving.py stderr:\n{result.stderr}")

        except Exception as e:
            self.get_logger().error(f"❌ Error ejecutando driving.py: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FireAlertSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
