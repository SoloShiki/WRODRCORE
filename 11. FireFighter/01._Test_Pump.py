# #!/usr/bin/env python3
# from gpiozero import Servo, DigitalOutputDevice
# from time import sleep, time

# # --- Pins ---
# SERVO_PIN = 24
# PUMP_PIN = 23

# # --- Motion parameters ---
# CYCLE_SPEED = 0.5        # seconds per motion step
# MOTION_DURATION = 10.0   # total duration
# ANGLE_150 = 0.5          # gpiozero servo value for ~150°
# ANGLE_170 = 0.7          # gpiozero servo value for ~170°
# ANGLE_NEUTRAL = 0.5      # neutral/center

# def main():
    # # --- Initialize hardware ---
    # servo = Servo(SERVO_PIN)
    # pump = DigitalOutputDevice(PUMP_PIN)

    # # Turn pump ON
    # pump.on()
    # print("💧 Pump ON. Spray motion started.")

    # start_time = time()
    # is_up_position = False  # start with 150°

    # try:
        # while (time() - start_time) < MOTION_DURATION:
            # # Alternate servo positions
            # if is_up_position:
                # servo.value = ANGLE_170
                # is_up_position = False
            # else:
                # servo.value = ANGLE_150
                # is_up_position = True

            # print(f"→ Servo moved to {servo.value}")
            # sleep(CYCLE_SPEED)

        # # Motion complete: center servo and turn off pump
        # print("✅ Motion complete. Pump OFF. Centering servo...")
        # servo.value = ANGLE_NEUTRAL
        # sleep(1)
        # pump.off()

    # except KeyboardInterrupt:
        # print("⚠️ Interrupted by user. Pump OFF.")
        # pump.off()

    # finally:
        # print("🧹 Done.")

# if __name__ == "__main__":
    # main()


#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState
from time import time

class SprayServoController(Node):
    def __init__(self):
        super().__init__('spray_servo_controller')

        # --- Parameters ---
        self.declare_parameter('servo_id', 3)
        self.declare_parameter('angle_150', 2000)   # servo position for ~150°
        self.declare_parameter('angle_170', 2300)   # servo position for ~170°
        self.declare_parameter('neutral', 2150)
        self.declare_parameter('cycle_speed', 0.5)  # seconds per motion step
        self.declare_parameter('motion_duration', 10.0)

        # --- Load parameters ---
        self.servo_id = self.get_parameter('servo_id').value
        self.angle_150 = self.get_parameter('angle_150').value
        self.angle_170 = self.get_parameter('angle_170').value
        self.neutral = self.get_parameter('neutral').value
        self.cycle_speed = self.get_parameter('cycle_speed').value
        self.motion_duration = self.get_parameter('motion_duration').value

        # --- Publisher ---
        self.publisher = self.create_publisher(SetPWMServoState,
                                               '/ros_robot_controller/pwm_servo/set_state', 10)

        # --- Control state ---
        self.is_up_position = False
        self.start_time = time()

        # --- Timer ---
        self.timer = self.create_timer(self.cycle_speed, self.timer_callback)

        self.get_logger().info("💧 Servo motion started between 150°–170°")

    def set_servo_position(self, position):
        msg = SetPWMServoState()
        servo_state = PWMServoState()
        servo_state.id = [self.servo_id]
        servo_state.position = [position]
        servo_state.offset = [0]
        msg.state.append(servo_state)
        self.publisher.publish(msg)
        self.get_logger().info(f"→ Servo moved to {position}")

    def timer_callback(self):
        elapsed = time() - self.start_time
        if elapsed > self.motion_duration:
            # Stop motion and center
            self.get_logger().info("✅ Motion complete. Centering servo...")
            self.set_servo_position(self.neutral)
            self.timer.cancel()
            return

        # Alternate servo position
        if self.is_up_position:
            self.set_servo_position(self.angle_150)
        else:
            self.set_servo_position(self.angle_170)
        self.is_up_position = not self.is_up_position

def main(args=None):
    rclpy.init(args=args)
    node = SprayServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⚠️ Interrupted by user. Centering servo...")
        node.set_servo_position(node.neutral)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
