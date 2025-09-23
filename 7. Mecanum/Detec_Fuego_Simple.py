#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading

class FireAlertSubscriber(Node):
    def __init__(self):
        super().__init__('fire_alert_subscriber')
        self.subscription = self.create_subscription(
            String,
            'fire_alert',
            self.listener_callback,
            10
        )
        self.get_logger().info('Subscriber listo para recibir alertas de fuego')

    def listener_callback(self, msg: String):
        self.get_logger().info(f'🔥 Alerta recibida: {msg.data}')

        # Detect fire keyword
        if "Fuego" in msg.data or "Alerta" in msg.data:
            self.get_logger().warn("🔥🔥 ¡FUEGO DETECTADO! Ejecutando acción...")

            # Launch an external script (non-blocking)
            threading.Thread(target=self.run_action, daemon=True).start()

    def run_action(self):
        """Custom action when fire is detected"""
        try:
            # Example: run a script or ROS2 node
            # Replace with your real command
            result = subprocess.run(
                ["echo", "🔥 Acción iniciada por fuego"],
                capture_output=True,
                text=True
            )
            self.get_logger().info(f"ACTION OUTPUT: {result.stdout.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error ejecutando acción: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FireAlertSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
